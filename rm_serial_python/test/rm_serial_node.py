import json                             # JSON序列化库

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型

from rm_interfaces.msg import ArmorsMsg, ArmorTracking  # 导入自定义消息类型
from rm_yolo_aim.armor_tracker import select_tracking_armor

class SerialNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化

        self.sub_tracker = self.create_subscription(
            ArmorTracking, '/tracker/target', self.listener_callback_tracker, 10)  # 订阅装甲板信息
        
        self.pub_uart_receive = self.create_publisher(String, '/uart/receive', 10)
        self.pub_uart_send    = self.create_publisher(String, '/uart/send', 10)
        

    def listener_callback_tracker(self, msg):
        try:
            # 将JSON格式的数据转换回Python字典
            yaw   = msg.yaw 
            pitch = msg.pitch
            deep  = msg.deep

            # 创建自定义消息对象并添加Header
            tracking_armor_msg = ArmorTracking()
            tracking_armor_msg.header = Header()  # 创建并设置Header
            tracking_armor_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
            tracking_armor_msg.header.frame_id = 'tracking_armor_frame'  # 可根据需要设置frame_id

            # 发送到串口

            # 发布消息
            self.pub_tracker.publish(tracking_armor_msg)
            self.get_logger().info(f'Published tracker message: {tracking_armor_msg.data}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {e}')


def main(args=None):                              # ROS2节点主入口main函数
    rclpy.init(args=args)                         # ROS2 Python接口初始化
    node = SerialNode("rm_serial_node")     # 创建ROS2节点对象
    rclpy.spin(node)                              # 循环等待ROS2退出
    node.destroy_node()                           # 销毁节点对象
    rclpy.shutdown()                              # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
