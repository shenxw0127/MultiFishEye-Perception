#!/usr/bin/env python3
"""IMU 椭球标定脚本。

从录制的 ROS2 bag（sensor_msgs/msg/Imu）中读取原始 IMU 数据，
对加速度计做椭球拟合（求 bias + 校正矩阵），对陀螺仪取静止段均值作为 bias，
输出 imu.yaml，格式与 src/oak/config/imu.yaml 一致。

标定模型（与 imu_correction.cpp 保持一致）：
    a_out = C · (a_raw - b_a)      # C 为 3x3 对称校正矩阵，b_a 为加速度计 bias
    w_out = w_raw - b_g            # b_g 为陀螺仪 bias

重要：录制 bag 时必须关闭 IMU 校正（imu_correction_enabled:=false），
否则拟合的是已校正数据，结果无意义。

用法:
    python3 calibrate_imu.py --bag ~/rosbags/fisheye4_xxx
    python3 calibrate_imu.py --bag <bag> --topic /imu --gravity 9.81 --output <path>
"""

import argparse
import os
import sys

import numpy as np


def find_default_output():
    """从当前工作目录向上查找源码树的 src/oak/config，写回源码而非 install 目录。"""
    d = os.getcwd()
    while True:
        candidate = os.path.join(d, "src", "oak", "config")
        if os.path.isdir(candidate):
            return os.path.join(candidate, "imu.yaml")
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    # 找不到工作区则退回当前目录
    return os.path.join(os.getcwd(), "imu.yaml")


def parse_args():
    default_output = find_default_output()
    parser = argparse.ArgumentParser(
        description="对录制的 ROS2 bag 中的 IMU 数据做椭球标定，输出 imu.yaml。"
    )
    parser.add_argument("--bag", required=True, help="ROS2 bag 目录路径（含 metadata.yaml）")
    parser.add_argument(
        "--topic", default="/oak/imu", help="IMU 话题名（默认 /oak/imu；旧版无命名空间的 bag 用 /imu）"
    )
    parser.add_argument(
        "--gravity", type=float, default=9.7936, help="当地重力加速度 m/s^2（默认 9.81）"
    )
    parser.add_argument(
        "--output", default=default_output, help="输出 yaml 路径（默认 src/oak/config/imu.yaml）"
    )
    parser.add_argument(
        "--static-accel-std",
        type=float,
        default=0.15,
        help="判定静止的加速度模长滑窗标准差阈值 m/s^2（默认 0.15）",
    )
    parser.add_argument(
        "--static-gyro-std",
        type=float,
        default=0.02,
        help="判定静止的角速度模长滑窗标准差阈值 rad/s（默认 0.02）",
    )
    parser.add_argument(
        "--window", type=int, default=40, help="静止判定滑窗样本数（默认 40，约 0.2s@200Hz）"
    )
    parser.add_argument(
        "--storage-id",
        default="",
        help="rosbag2 存储后端 id（如 mcap / sqlite3），留空则自动探测",
    )
    return parser.parse_args()


def read_imu(bag_path, topic, storage_id):
    """从 bag 中读取指定话题的 IMU 消息，返回 (accel Nx3, gyro Nx3)。"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"bag 路径不存在: {bag_path}")

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise KeyError(
            f"bag 中找不到话题 {topic}。可用话题: {sorted(topic_types.keys())}"
        )
    if topic_types[topic] != "sensor_msgs/msg/Imu":
        raise TypeError(
            f"话题 {topic} 类型为 {topic_types[topic]}，需要 sensor_msgs/msg/Imu"
        )

    msg_type = get_message(topic_types[topic])
    accel = []
    gyro = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        accel.append(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        )
        gyro.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    return np.asarray(accel, dtype=float), np.asarray(gyro, dtype=float)


def rolling_std_mask(mag, window, threshold):
    """返回布尔掩码：滑窗标准差 <= threshold 的样本视为静止。"""
    n = len(mag)
    if n < window:
        # 数据太短，用整体标准差判定
        return np.full(n, mag.std() <= threshold)
    mask = np.zeros(n, dtype=bool)
    half = window // 2
    # 用累积和高效计算滑窗均值/方差
    csum = np.concatenate([[0.0], np.cumsum(mag)])
    csum2 = np.concatenate([[0.0], np.cumsum(mag * mag)])
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, lo + window)
        lo = max(0, hi - window)
        cnt = hi - lo
        s = csum[hi] - csum[lo]
        s2 = csum2[hi] - csum2[lo]
        var = max(0.0, s2 / cnt - (s / cnt) ** 2)
        mask[i] = np.sqrt(var) <= threshold
    return mask


def fit_ellipsoid(points):
    """一般二次曲面最小二乘拟合。

    拟合 a x^2+b y^2+c z^2+2f yz+2g xz+2h xy+2p x+2q y+2r z = 1，
    返回椭球中心 center(3,) 和形状矩阵 A(3x3, 对称正定)，满足
    (X-center)^T A (X-center) = 1。
    """
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    D = np.column_stack(
        [x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y, 2 * x, 2 * y, 2 * z]
    )
    rhs = np.ones(len(points))
    # 最小二乘解 D @ v = 1
    v, *_ = np.linalg.lstsq(D, rhs, rcond=None)
    a, b, c, f, g, h, p, q, r = v

    # 二次型矩阵 (含常数项 -1)
    A4 = np.array(
        [
            [a, h, g, p],
            [h, b, f, q],
            [g, f, c, r],
            [p, q, r, -1.0],
        ]
    )
    A3 = A4[:3, :3]
    # 椭球中心: center = -A3^{-1} [p,q,r]
    center = np.linalg.solve(A3, -np.array([p, q, r]))
    # 平移到中心后的常数项
    const = A4[3, 3] + center @ np.array([p, q, r])
    # 中心化二次型: (X-c)^T (A3 / -const) (X-c) = 1
    shape = A3 / (-const)
    # 对称化
    shape = 0.5 * (shape + shape.T)
    return center, shape


def compute_correction(shape, gravity):
    """由椭球形状矩阵求校正矩阵 C，使 C·(a-bias) 落在半径 gravity 的球面上。

    椭球: (a-c)^T shape (a-c) = 1，即半轴由 shape 的特征值决定。
    令 M = sqrtm(shape)，则 M·(a-c) 落在单位球面。乘以 gravity 得半径 g 的球。
    返回 C = gravity * M（对称正定）。
    """
    # 对称矩阵特征分解求平方根
    eigvals, eigvecs = np.linalg.eigh(shape)
    if np.any(eigvals <= 0):
        raise ValueError(
            f"椭球拟合得到非正定形状矩阵（特征值 {eigvals}），"
            "数据可能未覆盖足够朝向或包含过多运动样本。"
        )
    sqrt_shape = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T
    C = gravity * sqrt_shape
    return 0.5 * (C + C.T)


def format_matrix(mat):
    return "\n".join(
        "    - [{}, {}, {}]".format(*[repr(float(x)) for x in row]) for row in mat
    )


def format_vector(vec):
    return "[{}, {}, {}]".format(*[repr(float(x)) for x in vec])


def main():
    args = parse_args()

    print(f"读取 bag: {args.bag}  话题: {args.topic}")
    accel, gyro = read_imu(args.bag, args.topic, args.storage_id)
    if len(accel) == 0:
        print("错误: 未读到任何 IMU 消息。", file=sys.stderr)
        return 1
    print(f"共读取 {len(accel)} 条 IMU 消息。")

    # --- 加速度计: 筛静止样本 + 椭球拟合 ---
    accel_mag = np.linalg.norm(accel, axis=1)
    accel_static = rolling_std_mask(accel_mag, args.window, args.static_accel_std)
    n_static = int(accel_static.sum())
    print(f"加速度计静止样本: {n_static} / {len(accel)}")
    if n_static < 50:
        print(
            "错误: 静止样本过少，无法可靠拟合椭球。请确保每个朝向静止保持数秒，"
            "或放宽 --static-accel-std。",
            file=sys.stderr,
        )
        return 1

    accel_pts = accel[accel_static]
    center, shape = fit_ellipsoid(accel_pts)
    C = compute_correction(shape, args.gravity)

    # 校正前后残差诊断
    corrected = (C @ (accel_pts - center).T).T
    corrected_mag = np.linalg.norm(corrected, axis=1)
    raw_res = accel_mag[accel_static] - args.gravity
    cor_res = corrected_mag - args.gravity
    print("加速度计标定诊断:")
    print(f"  bias (m/s^2): {center}")
    print(
        f"  校正前 |a| 均值={accel_mag[accel_static].mean():.4f} "
        f"残差RMS={np.sqrt(np.mean(raw_res**2)):.4f}"
    )
    print(
        f"  校正后 |a| 均值={corrected_mag.mean():.4f} "
        f"残差RMS={np.sqrt(np.mean(cor_res**2)):.4f} (目标 {args.gravity})"
    )

    # --- 陀螺仪: 取最静止段的均值 ---
    gyro_mag = np.linalg.norm(gyro, axis=1)
    gyro_static = rolling_std_mask(gyro_mag, args.window, args.static_gyro_std)
    # 加速度和陀螺同时静止的样本最可靠
    both_static = gyro_static & accel_static
    if both_static.sum() >= 50:
        gyro_bias = gyro[both_static].mean(axis=0)
        print(f"陀螺仪静止样本(与加速度同静止): {int(both_static.sum())}")
    elif gyro_static.sum() >= 50:
        gyro_bias = gyro[gyro_static].mean(axis=0)
        print(f"陀螺仪静止样本: {int(gyro_static.sum())}")
    else:
        gyro_bias = gyro.mean(axis=0)
        print("警告: 未找到足够的陀螺仪静止段，改用全体均值作为 bias。")
    print(f"  gyro bias (rad/s): {gyro_bias}")

    # --- 写 yaml ---
    content = (
        "accelerometer:\n"
        f"  bias: {format_vector(center)}\n"
        "  correction_matrix:\n"
        f"{format_matrix(C)}\n"
        "gyroscope:\n"
        f"  bias: {format_vector(gyro_bias)}\n"
    )
    output_path = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        f.write(content)
    print(f"\n已写入: {output_path}")
    print("-" * 40)
    print(content, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
