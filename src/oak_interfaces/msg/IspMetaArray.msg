# 一组同步帧（4 路）的 ISP 元数据，驱动在 /isp_meta 上随每组图像发布一条。
# 查看：ros2 topic echo /isp_meta
# Foxglove Plot 面板：/isp_meta.cameras[0].color_temperature_k 等
std_msgs/Header header           # stamp = 本组第一路相机的帧时间戳；frame_id = tf_prefix
IspMeta[] cameras                # 按 cam_board_sockets 顺序
