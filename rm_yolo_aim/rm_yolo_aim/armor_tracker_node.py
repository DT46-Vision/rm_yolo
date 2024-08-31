import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型
from rm_interfaces.msg import ArmorsMsg  # 导入自定义消息类型
import json                             # JSON序列化库

class ArmorTrackerNode(Node):
    def __init__(self, name):
        super().__init__(name)                               # ROS2节点父类初始化
        self.sub_armors = self.create_subscription(
            ArmorsMsg, '/detector/armors_info', self.listener_callback_armors, 10)  # 订阅装甲板信息
        self.sub_serial = self.create_subscription(
            String, '/serial_data', self.listener_callback_serial, 10)  # 订阅串口数据

    def listener_callback_armors(self, msg):
        # 获取消息头信息
        header = msg.header
        self.get_logger().info(f'Received armors message with timestamp: {header.stamp.sec}.{header.stamp.nanosec} and frame_id: {header.frame_id}')

        # 将JSON格式的数据转换回Python字典
        armors_dict = json.loads(msg.data)
        self.get_logger().info(f'Received armors data: {armors_dict}')

        # 这里可以对armors_dict进行进一步处理

    def listener_callback_serial(self, msg):
        # 获取串口数据
        serial_data = msg.data
        self.get_logger().info(f'Received serial data: {serial_data}')

        # 这里可以对串口数据进行进一步处理

def main(args=None):                              # ROS2节点主入口main函数
    rclpy.init(args=args)                         # ROS2 Python接口初始化
    node = ArmorTrackerNode("armor_tracker_node")     # 创建ROS2节点对象
    rclpy.spin(node)                              # 循环等待ROS2退出
    node.destroy_node()                           # 销毁节点对象
    rclpy.shutdown()                              # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
