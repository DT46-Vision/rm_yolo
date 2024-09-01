import serial
import struct
import time

# 定义 SendPacket 结构体的格式
# 格式说明：
# - B: uint8_t (1 byte)
# - ? : bool (1 byte)
# - f: float (4 bytes)
# - H: uint16_t (2 bytes)
packet_format = '<B?ffffH'  # 使用小端字节序

# 创建串口对象
ser = serial.Serial('COM3', 9600)  # 替换为你的串口号

# 准备数据
header = 0xA5
tracking = True  # 当前是否锁定目标
yaw = 45.0  # 目标在世界坐标系下的倾斜角度
v_yaw = 1.5  # 目标旋转的角速
pitch = 30.0  # 目标的俯仰角
deep = 100.0  # 深度
checksum = 0  # 校验和，后续可以计算

# 计算校验和（简单示例，实际应用中可能需要更复杂的算法）
checksum = header + (1 if tracking else 0) + int(yaw) + int(v_yaw) + int(pitch) + int(deep)

# 打包数据
packet = struct.pack(packet_format, header, tracking, yaw, v_yaw, pitch, deep, checksum)

# 发送数据
ser.write(packet)

# 等待一段时间以确保数据发送完成
time.sleep(1)

# 关闭串口
ser.close()