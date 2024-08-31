import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from std_msgs.msg import Header         # 头部消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # OpenCV图像处理库
import numpy as np                      # Python数值计算库
import json                             # JSON序列化库

from rm_yolo_aim.armor_detector import ArmorDetector
from rm_interfaces.msg import ArmorsMsg  # 导入自定义消息类型

# detector = ArmorDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')             # pt 原始模型
detector = ArmorDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best_openvino_model/')  # openvino 模型

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)   # 创建订阅者对象
        self.publisher_img  = self.create_publisher(Image, '/armors_img', 10)  # 创建图像发布者
        self.publisher_armors = self.create_publisher(ArmorsMsg, '/armors_info', 10)  # 创建串口信息发布者
        self.cv_bridge = CvBridge()                           # 创建图像转换对象

    def listener_callback(self, data):
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')    # 将ROS的图像消息转化成OpenCV图像
        img, armors_dict = detector.detect_armor(image)       # 检测图像，返回处理后的图像和装甲板信息字典

        # 发布处理后的图像
        result_img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
        self.publisher_img.publish(result_img_msg)
        # self.get_logger().info('Published processed image to /detector/result_img')

        # 将装甲板信息字典转换为JSON格式的字符串
        armors_json = json.dumps(armors_dict)

        # 创建自定义消息对象并添加Header
        armors_msg = ArmorsMsg()
        armors_msg.header = Header()  # 创建并设置Header
        armors_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        armors_msg.header.frame_id = 'armors_frame'  # 可根据需要设置frame_id

        # 设置JSON格式的装甲板信息
        armors_msg.data = armors_json

        # 发布消息
        self.publisher_armors.publish(armors_msg)
        self.get_logger().info(f'Published UART message: {armors_msg.data}')

def main(args=None):                            # ROS2节点主入口main函数
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("armor_detector_node")       # 创建ROS2节点对象
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
