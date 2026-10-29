# 单路 OAK 相机一帧的 ISP（3A）元数据，直接取自 DepthAI ImgFrame 的 CameraSettings。
# 自动曝光 / 自动白平衡模式下这就是固件当前实际用的值，可用来观察 AE/AWB 收敛情况、
# 或为切回手动模式（manual_exposure_us / manual_iso / manual_wb_kelvin）挑一个固定值。
string camera                    # 板上插座名，如 CAM_A
builtin_interfaces/Time stamp    # 该帧图像的 header.stamp，与 /<camera>/image 一一对应
int64 sequence_num               # 设备端帧序号
uint32 exposure_time_us          # 曝光时间（微秒）
int32 iso                        # 感光度（ISO）
int32 color_temperature_k        # AWB 色温（K）；MonoCamera 没有白平衡，固件填固定值 6500
int32 lens_position              # 对焦马达位置 0..255；定焦镜头为 0 或 -1
