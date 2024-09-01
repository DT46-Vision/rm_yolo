import json                             # JSON序列化库

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型

from rm_interfaces.msg import ArmorsMsg, ArmorTracking  # 导入自定义消息类型
from rm_yolo_aim.armor_tracker import select_tracking_armor, pixel_to_angle_and_deep

class ArmorTrackerNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化

        self.sub_armors = self.create_subscription(
            ArmorsMsg, '/detector/armors_info', self.listener_callback_armors, 10)  # 订阅装甲板信息
        
        self.sub_serial = self.create_subscription(
            String, '/uart/receive', self.listener_callback_serial, 10)  # 订阅串口数据
        
        self.pub_tracker = self.create_publisher(ArmorTracking, '/tracker/target', 10)
        
        self.tracking_color = None
        self.tracking_armor = None

    def listener_callback_armors(self, msg):
        try:
            # 将JSON格式的数据转换回Python字典
            armors_dict = json.loads(msg.data)
            # self.get_logger().info(f'Received armors data: {armors_dict}')

            # 选择要跟踪的装甲板
            self.tracking_armor = select_tracking_armor(armors_dict, 0)  # 0表示红色
            self.get_logger().info(f"get tracking_armor {self.tracking_armor}")

            yaw, pitch, deep = pixel_to_angle_and_deep(self.tracking_armor, 72) 

            self.get_logger().info(f"yaw, pitch, deep: {yaw, pitch, deep}")
            
            # 将装甲板信息字典转换为msg消息定义的格式
            tracking_armor_json = json.dumps(self.tracking_armor)

            # 创建自定义消息对象并添加Header
            tracking_armor_msg = ArmorTracking()
            tracking_armor_msg.header = Header()  # 创建并设置Header
            tracking_armor_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
            tracking_armor_msg.header.frame_id = 'tracking_armor_frame'  # 可根据需要设置frame_id

            # 设置的装甲板信息
            tracking_armor_msg.data  = tracking_armor_json
            tracking_armor_msg.yaw   = float(yaw)
            tracking_armor_msg.pitch = float(pitch)
            tracking_armor_msg.deep  = float(deep)

            # 发布消息
            self.pub_tracker.publish(tracking_armor_msg)
            # self.get_logger().info(f'Published tracker message: {tracking_armor_msg.data}')
            # self.get_logger().info(f"Preparing to publish: yaw={tracking_armor_msg.yaw}, pitch={tracking_armor_msg.pitch}, deep={tracking_armor_msg.deep}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {e}')


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
