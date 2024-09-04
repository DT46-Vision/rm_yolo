import serial
import struct
import time

class SerialPacketSender:
    def __init__(self, port, baudrate, header=0xA5, tracking=True, yaw=45.0, v_yaw=1.5, pitch=30.0, deep=100.0):
        self.ser = serial.Serial(port, baudrate)
        self.packet_format = '<B?ffffH'  # 使用小端字节序
        self.header = header
        self.tracking = tracking
        self.yaw = yaw
        self.v_yaw = v_yaw
        self.pitch = pitch
        self.deep = deep

    def create_packet(self):
        checksum = self.header + (1 if self.tracking else 0) + int(self.yaw) + int(self.v_yaw) + int(self.pitch) + int(self.deep)
        # 打包数据
        return struct.pack(self.packet_format, self.header, self.tracking, self.yaw, self.v_yaw, self.pitch, self.deep, checksum)

    def send_packet(self):
        packet = self.create_packet()
        self.ser.write(packet)
        time.sleep(1)  # 等待一段时间以确保数据发送完成

    def close(self):
        self.ser.close()

# 使用示例
if __name__ == "__main__":
    sender = SerialPacketSender('COM3', 9600, header=0xA5, tracking=True, yaw=45.0, v_yaw=1.5, pitch=30.0, deep=100.0)
    sender.send_packet()
    sender.close()