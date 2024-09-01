import time
import serial
import threading
import struct
import zlib

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header  # 字符串消息类型和头部消息类型
from rm_interfaces.msg import ArmorTracking # , SerialReceive # 导入自定义消息类型


class RMSerialDriver(Node):
    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info("启动 RMSerialDriver!")

        # 获取参数
        self.get_params()

        # 创建订阅者
        self.sub_tracker = self.create_subscription(
            ArmorTracking, "/tracker/target", self.send_data, 10
        )

        # 创建发布者
        # self.pub_uart_receive = self.create_publisher(String, "/uart/receive", 10)
        
        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=self.device_name,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1,
            )
            if self.serial_port.is_open:
                self.get_logger().info("创建串口 successfully.")
                # self.receive_thread = threading.Thread(target=self.receive_data)
                # self.receive_thread.start()

        except serial.SerialException as e:
            self.get_logger().error(f"创建串口时出错: {self.device_name} - {str(e)}")
            raise e

    def get_params(self):
        """获取并设置串口相关的参数"""
        self.device_name  = self.declare_parameter("device_name", "/dev/ttyUSB0").value
        self.baud_rate    = self.declare_parameter("baud_rate", 115200).value
        self.flow_control = self.declare_parameter("flow_control", "none").value
        self.parity       = self.declare_parameter("parity", "none").value
        self.stop_bits    = self.declare_parameter("stop_bits", "1").value

    # def process_packet(self, packet):
    #     """处理接收到的包数据"""
    #     # 校验并处理 CRC (这里可以加上 CRC 校验)
    #     detect_color = packet[1]

    #     # 更新目标颜色参数
    #     if detect_color != self.tracking_color:
    #         self.tracking_color = detect_color
        
    # def receive_data(self):
    #     """接收串口数据并处理"""
    #     while rclpy.ok():
    #         try:
    #             # 读取数据头部
    #             header = self.serial_port.read(1)
    #             if header and header[0] == 0x5A:
    #                 data = self.serial_port.read(29)

    #                 if len(data) == 29:
    #                     packet = struct.unpack("<Bffffff?", header + data)
    #                     self.process_packet(packet)

    #                 else:
    #                     self.get_logger().warn("Received data length mismatch")
    #             else:
    #                 self.get_logger().warn("Invalid header received")

    #         except serial.SerialException as e:
    #             self.get_logger().error(f"接收数据时出错: {str(e)}")
    #             self.reopen_port()

    def send_data(self, msg):
        """处理目标信息并通过串口发送"""
        try:
            id_map =   ["B1", "B2", "B3", "B4", "B5", "B7", 
                        "R1", "R2", "R3", "R4", "R5", "R7"]
            
            self.get_logger().info(f"发送数据: {msg}")

            header = 0x5A

            packet = struct.pack(
                "<Bfff",
                header,
                msg.yaw,
                msg.pitch,
                msg.deep,
            )

            # 计算校验和
            checksum = zlib.crc32(packet) & 0xFFFF  # 取低16位作为校验和

            # 将校验和添加到数据包中
            packet_with_checksum = packet + struct.pack("<H", checksum)

            self.get_logger().info(f"打包后的数据: {packet_with_checksum}")

            self.serial_port.write(packet_with_checksum)

        except Exception as e:
            self.get_logger().error(f"发送数据时出错: {str(e)}")
            self.reopen_port()

    def reopen_port(self):
        """重新打开串口"""
        self.get_logger().warn("尝试重新打开串口")
        try:
            if self.serial_port.is_open:
                self.serial_port.close()

            self.serial_port.open()
            self.get_logger().info("成功重新打开串口")
        except Exception as e:
            self.get_logger().error(f"重新打开串口时出错: {str(e)}")
            time.sleep(1)
            self.reopen_port()


def main(args=None):  # ROS2节点主入口main函数
    rclpy.init(args=args)  # ROS2 Python接口初始化
    node = RMSerialDriver("rm_serial_python")  # 创建ROS2节点对象
    rclpy.spin(node)  # 循环等待ROS2退出
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS2 Python接口


if __name__ == "__main__":
    main()

# def main(args=None):
#     rclpy.init(args=args)
#     rm_serial_driver = RMSerialDriver()

#     executor = MultiThreadedExecutor()
#     executor.add_node(rm_serial_driver)

#     try:
#         executor.spin()
#     finally:
#         rm_serial_driver.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
