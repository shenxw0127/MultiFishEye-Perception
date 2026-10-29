#!/usr/bin/env python3
"""联合录制协调节点：一个 /record_all 请求同时触发 OAK / RealSense / VRPN 录制，bag 放进同一个 session 目录。

    ros2 service call /record_all oak_interfaces/srv/Record "{duration_s: 30}"
    -> <bag_root>/session_<时间戳>/oak    （4 路鱼眼 JPEG @record_hz + IMU）
       <bag_root>/session_<时间戳>/d435i  （RGB / 深度 / IMU / 外参 / tf_static）
       <bag_root>/session_<时间戳>/vrpn   （动捕刚体位姿，vrpn:=false 时没有）

各路录制是独立进程各自写 bag（大图像不跨进程传），开始/结束时刻相差几 ms，离线按重叠区间裁剪。
时间对齐靠 header.stamp：两边都是"采集时刻 + 主机系统时钟"，按最近时间戳配对即可（见 record_all.launch.py）。
bag_name 若在请求里给出则作为 session 目录名。
"""

import threading
from datetime import datetime

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from oak_interfaces.srv import Record

SERVICE_TIMEOUT_S = 5.0


class RecordAll(Node):
    def __init__(self):
        super().__init__("record_all")
        # 子录制服务：{bag 子目录名: 服务名}
        self.declare_parameter("targets", ["oak:/oak/record", "d435i:/d435i/record"])
        targets = self.get_parameter("targets").value
        group = ReentrantCallbackGroup()
        self.clients_ = {}
        for item in targets:
            name, _, service = item.partition(":")
            if not name or not service:
                raise ValueError("targets entries must be '<subdir>:<service>', got '{}'".format(item))
            self.clients_[name] = self.create_client(Record, service, callback_group=group)
        self.create_service(Record, "/record_all", self.on_request, callback_group=group)
        self.get_logger().info("record_all ready: {}".format(", ".join(targets)))

    def on_request(self, request, response):
        session = request.bag_name or "session_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        missing = [n for n, c in self.clients_.items() if not c.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S)]
        if missing:
            response.success = False
            response.message = "recorder service not available: " + ", ".join(missing)
            return response

        # 并行发出，尽量让两路同时开录
        futures = {}
        for name, client in self.clients_.items():
            sub = Record.Request(duration_s=request.duration_s, bag_name="{}/{}".format(session, name))
            futures[name] = client.call_async(sub)
        done = threading.Event()
        pending = [len(futures)]
        lock = threading.Lock()

        def on_done(_):
            with lock:
                pending[0] -= 1
                if pending[0] == 0:
                    done.set()

        for future in futures.values():
            future.add_done_callback(on_done)
        done.wait(timeout=SERVICE_TIMEOUT_S)

        results = []
        paths = []
        ok = True
        for name, future in futures.items():
            result = future.result() if future.done() else None
            if result is None:
                ok = False
                results.append("{}: no response".format(name))
            else:
                ok = ok and result.success
                results.append("{}: {}".format(name, result.message))
                if result.bag_path:
                    paths.append(result.bag_path)
        response.success = ok
        response.message = "; ".join(results)
        # 各子 bag 同在 session 目录下，返回其公共父目录
        response.bag_path = paths[0].rsplit("/", 1)[0] if paths else ""
        log = self.get_logger().info if ok else self.get_logger().error
        log("record_all {:.1f}s -> {} ({})".format(request.duration_s, response.bag_path, response.message))
        return response


def main():
    rclpy.init()
    node = RecordAll()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
