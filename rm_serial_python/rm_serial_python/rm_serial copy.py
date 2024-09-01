import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from auto_aim_interfaces.msg import Target
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import serial
import threading
import struct
import time

class RMSerialDriver(Node):
    def __init__(self):
        super().__init__('rm_serial_driver')

        self.get_logger().info("启动 RMSerialDriver!")

        # 获取参数
        self.get_params()

        # 初始化 TF 广播器
        self.timestamp_offset = self.declare_parameter('timestamp_offset', 0.0).value
        self.tf_broadcaster = TransformBroadcaster(self)

        # 创建发布者
        self.latency_pub = self.create_publisher(Float64, '/latency', 10)
        self.marker_pub = self.create_publisher(Marker, '/aiming_point', 10)

        # 初始化其他变量
        self.aiming_point = Marker()
        self.aiming_point.header.frame_id = 'odom'
        self.aiming_point.ns = 'aiming_point'
        self.aiming_point.type = Marker.SPHERE
        self.aiming_point.action = Marker.ADD
        self.aiming_point.scale.x = self.aiming_point.scale.y = self.aiming_point.scale.z = 0.12
        self.aiming_point.color.r = 1.0
        self.aiming_point.color.g = 1.0
        self.aiming_point.color.b = 1.0
        self.aiming_point.color.a = 1.0
        self.aiming_point.lifetime = rclpy.duration.Duration(seconds=0.1)

        # 创建订阅者
        self.target_sub = self.create_subscription(
            Target, '/tracker/target', self.send_data, 10)

        # 初始化串口
        try:
            self.serial_port = serial.Serial(
                port=self.device_name,
                baudrate=self.baud_rate,
                timeout=1,
                write_timeout=1
            )
            if self.serial_port.is_open:
                self.receive_thread = threading.Thread(target=self.receive_data)
                self.receive_thread.start()
        except serial.SerialException as e:
            self.get_logger().error(f"创建串口时出错: {self.device_name} - {str(e)}")
            raise e

    def get_params(self):
        """获取并设置串口相关的参数"""
        self.device_name = self.declare_parameter('device_name', '').value
        self.baud_rate = self.declare_parameter('baud_rate', 9600).value
        self.flow_control = self.declare_parameter('flow_control', 'none').value
        self.parity = self.declare_parameter('parity', 'none').value
        self.stop_bits = self.declare_parameter('stop_bits', '1').value

    def receive_data(self):
        """接收串口数据并处理"""
        while rclpy.ok():
            try:
                # 读取数据头部
                header = self.serial_port.read(1)

                if header and header[0] == 0x5A:
                    data = self.serial_port.read(29)
                    if len(data) == 29:
                        packet = struct.unpack('<Bffffff?', header + data)
                        self.process_packet(packet)
                    else:
                        self.get_logger().warn('Received data length mismatch')
                else:
                    self.get_logger().warn('Invalid header received')
            except serial.SerialException as e:
                self.get_logger().error(f"接收数据时出错: {str(e)}")
                self.reopen_port()

    def send_data(self, msg):
        """处理目标信息并通过串口发送"""
        try:
            id_map = {
                "": 0, "outpost": 0, "1": 1, "2": 2,
                "3": 3, "4": 4, "5": 5, "guard": 6, "base": 7
            }

            packet = struct.pack(
                '<Bffffff?',
                msg.tracking,
                id_map.get(msg.id, 0),
                msg.armors_num,
                msg.position.x,
                msg.position.y,
                msg.position.z,
                msg.yaw
            )

            self.serial_port.write(packet)

            # 计算并发布延迟
            latency = Float64()
            latency.data = (self.get_clock().now().to_msg().sec - msg.header.stamp.sec) * 1000.0
            self.get_logger().debug(f"Total latency: {latency.data}ms")
            self.latency_pub.publish(latency)

        except Exception as e:
            self.get_logger().error(f"发送数据时出错: {str(e)}")
            self.reopen_port()

    def process_packet(self, packet):
        """处理接收到的包数据"""
        # 校验并处理 CRC (这里可以加上 CRC 校验)
        detect_color = packet[1]

        # 更新目标颜色参数
        if not hasattr(self, 'previous_receive_color') or detect_color != self.previous_receive_color:
            self.set_parameters([Parameter('detect_color', Parameter.Type.INTEGER, detect_color)])
            self.previous_receive_color = detect_color

        # 处理复位跟踪器请求
        if packet[-1]:
            self.reset_tracker()

        # 广播TF变换
        self.broadcast_transform(packet)

        # 如果目标位置有效，发布标记
        if abs(packet[2]) > 0.01:
            self.aiming_point.header.stamp = self.get_clock().now().to_msg()
            self.aiming_point.pose.position.x = packet[2]
            self.aiming_point.pose.position.y = packet[3]
            self.aiming_point.pose.position.z = packet[4]
            self.marker_pub.publish(self.aiming_point)

    def broadcast_transform(self, packet):
        """广播 TF 变换"""
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "odom"
        transform_stamped.child_frame_id = "gimbal_link"

        # 四元数转换 (roll, pitch, yaw)
        q = tf_transformations.quaternion_from_euler(packet[5], packet[6], packet[7])
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform_stamped)

    def reset_tracker(self):
        """重置跟踪器"""
        if not hasattr(self, 'reset_tracker_client'):
            self.reset_tracker_client = self.create_client(Trigger, '/tracker/reset')

        if self.reset_tracker_client.service_is_ready():
            request = Trigger.Request()
            self.reset_tracker_client.call_async(request)
            self.get_logger().info("Reset tracker!")

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


def main(args=None):
    rclpy.init(args=args)
    rm_serial_driver = RMSerialDriver()

    executor = MultiThreadedExecutor()
    executor.add_node(rm_serial_driver)

    try:
        executor.spin()
    finally:
        rm_serial_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
