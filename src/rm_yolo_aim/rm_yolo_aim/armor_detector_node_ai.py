# armor_detector_node.py
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header         # 头部消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类

import json                             # JSON序列化库
import os

from rm_yolo_aim.armor_detector_ai import ArmorDetector
from rm_interfaces.msg import ArmorsMsg  # 导入自定义消息类型

USER = os.environ['USER']
detector = ArmorDetector()  # openvino 模型
# detector = ArmorDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')             # pt 原始模型
detector.model_path = (
        f"/home/{USER}/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best_openvino_model/"
    )

class ArmorDetectorNode(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)   # 创建订阅者对象
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera_info', self.listener_callback_camera_info, 10)
        
        self.publisher_undistorted_img = self.create_publisher(Image, '/detector/undistorted_img', 10)
        self.publisher_img  = self.create_publisher(Image, '/detector/armors_img', 10)  # 创建图像发布者
        self.publisher_armors = self.create_publisher(ArmorsMsg, '/detector/armors_info', 10)  # 创建串口信息发布者
        self.cv_bridge = CvBridge()                           # 创建图像转换对象

        self.camera_info = None

    def listener_callback_camera_info(self, data):
        if self.camera_info != data:
            self.camera_info = data

    def listener_callback(self, data):
        cv_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')    # 将ROS的图像消息转化成OpenCV图像

        try:
            tmp = len(self.camera_info.d)
            if tmp != 0:
                cv_image = detector.undistort_image(cv_image, self.camera_info)  # 畸变校正
                self.get_logger().info('畸变校正了图像')

        except AttributeError as e:
            self.get_logger().info(e)
        

        img, armors_dict = detector.detect_armor(cv_image)       # 检测图像，返回处理后的图像和装甲板信息字典

        # 发布处理后的图像
        undistorted_img_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        undistorted_img_msg.header.frame_id = "undistorted_img_frame"
        self.publisher_undistorted_img.publish(undistorted_img_msg)

        result_img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
        result_img_msg.header.frame_id = "camera_optical_frame"
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
        self.get_logger().info(f'发布了 armors 的数据: {armors_msg.data}')

def main(args=None):                            # ROS2节点主入口main函数
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ArmorDetectorNode("armor_detector_node")       # 创建ROS2节点对象
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
