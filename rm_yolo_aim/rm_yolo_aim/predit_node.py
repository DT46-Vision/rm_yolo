#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅图像话题
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库

from rm_yolo_aim.predit import YOLOObjectDetector

detector = YOLOObjectDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')

"""
创建一个订阅者节点
"""
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)   # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()                           # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')     # 输出日志信息，提示已进入回调函数
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')  # 将ROS的图像消息转化成OpenCV图像
        # self.object_detect(image)                           # 苹果检测
        img, max_center = detector.predict(image)

        # self.get_logger().info(f"max_center: {max_center}")

def main(args=None):                            # ROS2节点主入口main函数
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("predit_node")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口
