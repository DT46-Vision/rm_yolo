#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅图像话题并发布处理结果
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from std_msgs.msg import String         # 字符串消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库

from rm_yolo_aim.predit import YOLOObjectDetector

detector = YOLOObjectDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)   # 创建订阅者对象
        self.publisher_img = self.create_publisher(Image, '/detector/result_img', 10)  # 创建图像发布者
        self.publisher_uart = self.create_publisher(String, '/uart_msg_send', 10)     # 创建串口信息发布者
        self.cv_bridge = CvBridge()                           # 创建图像转换对象

    def listener_callback(self, data):
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')    # 将ROS的图像消息转化成OpenCV图像
        img, uart_msg = detector.predict(image)               # 检测图像

        # 发布处理后的图像
        result_img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
        self.publisher_img.publish(result_img_msg)
        self.get_logger().info('Published processed image to /detector/result_img')

        # 发布串口信息
        uart_msg_msg = String(data=str(uart_msg))
        self.publisher_uart.publish(uart_msg_msg)
        self.get_logger().info(f'Published UART message: {uart_msg}')

def main(args=None):                            # ROS2节点主入口main函数
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("predit_node")  # 创建ROS2节点对象
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()