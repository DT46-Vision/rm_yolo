# armor_detector_opencv_node.py
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header         # 头部消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import json                             # JSON序列化库
from rcl_interfaces.msg import SetParametersResult  # 导入 SetParametersResult 消息类型
from rm_yolo_aim.armor_detector_opencv import ArmorDetector
from rm_interfaces.msg import ArmorsMsg, Decision  # 导入自定义消息类型
import time

# 模式参数字典
detect_color =  2  # 颜色参数 0: 识别红色装甲板, 1: 识别蓝色装甲板, 2: 识别全部装甲板
display_mode = 0 # 显示模式 None: 不显示, Binary: 显示二值化图, All: 显示二值化图和结果图像
# night :16, 15
# day :45, 45
binary_val_blue = 25
binary_val_red = 24
light_params = {
    "light_area_min": 5,  # 最小灯条面积
    "light_angle_min": -35,  # 最小灯条角度
    "light_angle_max": 35,  # 最大灯条角度
    "light_red_ratio": 3.5,
    "light_blue_ratio": 3.5,
    "light_angle_tol": 7,  # 灯条角度容差
    "vertical_discretization": 1.5,  # 垂直离散
    "height_tol": 18,  # 高度容差
    "cy_tol": 5,  # 中心点的y轴容差
    "height_multiplier": 2.7, 
    "hw" : 2.0 
}

# 颜色参数字典
color_params = {
    "armor_color": {1: (255, 255, 0), 0: (128, 0, 128)},  # 装甲板颜色映射
    "light_color": {1: (200, 71, 90), 0: (0, 100, 255)},  # 灯条颜色映射
    "light_dot": {1: (0, 0, 255), 0: (255, 0, 0)}  # 灯条中心点颜色映射
}

detector = ArmorDetector(detect_color, display_mode, binary_val_red, light_params, color_params)  # 创建检测器对象

def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为纳秒。"""
    current_time = time.time_ns()  # 获取当前时间（单位：纳秒）

    if last_time[0] is None:  # 如果是第一次调用，更新last_time
        last_time[0] = current_time
        return 1  # 防止除零错误，返回1纳秒

    else:  # 计算时间差
        diff = current_time - last_time[0]  # 计算时间差（单位：纳秒）
        last_time[0] = current_time  # 更新上次调用时间
        return diff / 1e6 # 返回时间差（秒）

def time_logger(func):  # 定义装饰器
    def wrapper(*args, **kwargs):  # 包装函数
        start_time = time.time_ns()  # 记录开始时间
        result = func(*args, **kwargs)  # 调用原函数
        end_time = time.time_ns()  # 记录结束时间
        dt = end_time - start_time
        print(f"函数 '{func.__name__}' 的运行时间: {dt/1e6} ms, fps: {1/(dt/1e9)} ")  # 打印运行时间
        return result  # 返回原函数的结果
    return wrapper


class ArmorDetectorNode(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)   # 创建订阅者对象
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera_info', self.listener_callback_camera_info, 10)
        self.sub_serial = self.create_subscription(
            Decision, '/nav/decision', self.listener_callback_serial, 10)  # 订阅串口数据
        
        self.publisher_binary_img  = self.create_publisher(Image, '/detector/binary_img', 10)  # 创建图像发布者
        self.publisher_img  = self.create_publisher(Image, '/detector/armors_img', 10)  # 创建图像发布者
        self.publisher_armors = self.create_publisher(ArmorsMsg, '/detector/armors_info', 10)  # 创建串口信息发布者
        self.cv_bridge = CvBridge()                           # 创建图像转换对象
        self.cv_image = None
        self.camera_info = None
        self.tracking_color = -1

        # 在节点初始化中声明参数
        for key, value in light_params.items():
            self.declare_parameter(key, value)  # 声明灯条参数
        # 声明 binary_val 和 detect_color 参数并添加回调
        self.declare_parameter('binary_val', detector.binary_val)  # 声明 binary_val 参数
        self.declare_parameter('detect_color', detector.color)  # 声明 detect_color 参数
        self.declare_parameter('display_mode', detector.display_mode)  # 声明 display_mode 参数        
        self.add_on_set_parameters_callback(self.param_callback)  # 添加参数回调
        self.get_logger().info('Armor Detector Node has started.')

    def param_callback(self, params):  # 参数回调函数
        for param in params:
            if param.name == 'binary_val':  # 检查 binary_val 参数
                detector.binary_val = param.value  # 更新 binary_val
                self.get_logger().info(f'更新 binary_val: {detector.binary_val}')  # 打印更新信息
            elif param.name == 'detect_color':  # 检查 detect_color 参数
                detector.color = param.value  # 更新 detect_color
                self.get_logger().info(f'更新 detect.color: {detector.color}')  # 打印更新信息
            elif param.name == 'display_mode':  # 检查 display_mode 参数
                detector.display_mode = param.value  # 更新 display_mode
                self.get_logger().info(f'更新 display_mode: {detector.display_mode}')  # 打印更新信息
            elif param.name in light_params:  # 检查灯条参数
                detector.light_params[param.name] = param.value  # 更新灯条参数
                self.get_logger().info(f'更新灯条参数 {param.name}: {light_params[param.name]}')  # 打印更新信息
        return SetParametersResult(successful=True)  # 返回成功结果
    
    def listener_callback_serial(self, msg):
        # 获取 Decision 数据
        # self.get_logger().info(f'Received Decision data: {msg}')

        # 这里可以对串口数据进行进一步处理
        if self.tracking_color != msg.color:
            self.tracking_color = msg.color

            if self.tracking_color == 0:
                detector.binary_val = binary_val_red
                self.get_logger().warn(f'二值化阈值改变为 {detector.binary_val},颜色改变为红色')
            elif self.tracking_color == 1:
                self.tracking_color = binary_val_blue
                self.get_logger().warn(f'二值化阈值改变为 {detector.binary_val},颜色改变为蓝色')
            else:
                self.get_logger().warn(f'颜色不合法')


    def listener_callback_camera_info(self, data):
        if self.camera_info != data:
            self.camera_info = data

    @time_logger
    def listener_callback(self, data):
        # dt = time_diff()
        # print(f'last，时间 {dt}')
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')    # 将ROS的图像消息转化成OpenCV图像

        cv_image = self.cv_image

        # dt = time_diff()
        # print(f'消息转化成OpenCV图像，时间 {dt}')

        # try:
        #     tmp = len(self.camera_info.d)
        #     if tmp != 0:
        #         cv_image = detector.undistort_image(cv_image, self.camera_info)  # 畸变校正
        #         self.get_logger().info('畸变校正了图像')

        # except AttributeError as e:
        #     self.get_logger().info(e)


        # dt = time_diff()
        # print(f'畸变校正，时间 {dt}')

        armors_dict = detector.detect_armor(cv_image)       # 检测图像，返回处理后的图像和装甲板信息字典

        # dt = time_diff()
        # print(f'检测，时间 {dt}')
        
        if detector.display_mode > 0 :
            img_binary, result_img = detector.display()

            binary_img_msg = self.cv_bridge.cv2_to_imgmsg(img_binary, 'mono8')
            binary_img_msg.header.frame_id = "binary_img_frame"
            self.publisher_binary_img.publish(binary_img_msg)

            if detector.display_mode > 1 :
                result_img_msg = self.cv_bridge.cv2_to_imgmsg(result_img, 'bgr8')
                result_img_msg.header.frame_id = "camera_optical_frame"
                self.publisher_img.publish(result_img_msg)
        
        # dt = time_diff()
        # print(f'display，时间 {dt}')

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
        # dt = time_diff()
        # print(f'发布消息，时间 {dt}')


def main(args=None):                            # ROS2节点主入口main函数
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ArmorDetectorNode("armor_detector_opencv_node")       # 创建ROS2节点对象
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()