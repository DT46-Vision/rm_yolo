import json
import time                             # JSON序列化库
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型
from sensor_msgs.msg import Image       # 图像消息类型
from rm_interfaces.msg import ArmorsMsg, ArmorTracking, Decision  # 导入自定义消息类型
from rm_yolo_aim.armor_tracker_new import select_tracking_armor, pixel_to_angle_and_deep
from rcl_interfaces.msg import SetParametersResult  # 导入 SetParametersResult 消息类型
from rm_yolo_aim.Kalman import KalmanFilter
from loguru import logger
import time  # 导入时间模块


def time_diff(last_time=[None]):
    """计算两次调用之间的时间差，单位为纳秒。"""
    current_time = time.time_ns()  # 获取当前时间（单位：纳秒）

    if last_time[0] is None:  # 如果是第一次调用，更新last_time
        last_time[0] = current_time
        return 1  # 防止除零错误，返回1纳秒

    else:  # 计算时间差
        diff = current_time - last_time[0]  # 计算时间差（单位：纳秒）
        last_time[0] = current_time  # 更新上次调用时间
        return diff / 1e9  # 返回时间差（秒）
    
class ArmorTrackerNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化

        self.sub_armors = self.create_subscription(
            ArmorsMsg, '/detector/armors_info', self.listener_callback_armors, 10)  # 订阅装甲板信息
        self.sub_cam    = self.create_subscription(
            Image, 'image_raw', self.listener_callback_cam, 10)

        self.sub_serial = self.create_subscription(
            Decision, '/nav/decision', self.listener_callback_serial, 10)  # 订阅串口数据

        self.pic_width = 1024       # 随便初始化一个图像宽度
        self.center_last = (0, 0)   # 默认初始化中心点坐标为(0, 0)
        self.height_last = 0             # 初始化armmor高度为0

        self.use_kf = True          # 是否使用卡尔曼滤波
        self.kf_cx = KalmanFilter()
        self.kf_cy = KalmanFilter()
        self.kf_h = KalmanFilter()

        self.lost = 0               # 初始化丢失帧数
        self.frame_add = 45         # 初始化补帧数
        self.reflection_hight_tol = 20
        self.reflection_cx_tol = 10
        self.offset_yaw = 0.0
        self.offset_pitch = 0.0
        self.deep_buff = 0.000_001

        self.pub_tracker = self.create_publisher(ArmorTracking, '/tracker/target', 10) # 创建发布者/tracker/target

        self.tracking_color = -1    # 1蓝色表示, 0表示红色, 现初始化为红色
        self.follow_decision = 1
        self.tracking_armor = None # 初始化追踪装甲板为None

        self.declare_parameter('use_kf', self.use_kf)  # 声明 use_kf 参数
        self.declare_parameter('frame_add', self.frame_add)  # 声明 frame_add 参数
        self.declare_parameter('follow_decision', self.follow_decision)  # 声明 frame_add 参数
        self.declare_parameter('tracking_color', self.tracking_color)  # 声明 detect_color 参数
        self.declare_parameter('reflection_hight_tol', self.reflection_hight_tol)  # 声明 frame_add 参数
        self.declare_parameter('reflection_cx_tol', self.reflection_cx_tol)  # 声明 detect_color 参数
        self.declare_parameter('offset_yaw', self.offset_yaw)  # 声明 detect_color 参数
        self.declare_parameter('offset_pitch', self.offset_pitch)  # 声明 detect_color 参数
        self.declare_parameter('deep_buff', self.deep_buff)  # 声明 detect_color 参数
        self.add_on_set_parameters_callback(self.param_callback)  # 添加参数回调
        self.get_logger().info('Armor Tracker Node has started.')

    def param_callback(self, params):
        for param in params:
            if param.name == 'use_kf':
                self.use_kf = param.value
            if param.name == 'frame_add':
                self.frame_add = param.value
            if param.name == 'tracking_color':
                self.tracking_color = param.value
            if param.name == 'follow_decision':
                self.follow_decision = param.value
            if param.name == 'reflection_hight_tol':
                self.reflection_hight_tol = param.value
            if param.name == 'reflection_cx_tol':
                self.reflection_cx_tol = param.value
            if param.name == 'offset_yaw':
                self.offset_yaw = param.value
            if param.name == 'offset_pitch':
                self.offset_pitch = param.value
            if param.name == 'deep_buff':
                self.deep_buff = param.value
        return SetParametersResult(successful=True)  # 返回成功结果

    def listener_callback_cam(self, data):
        if self.pic_width != data.width:
            self.pic_width = data.width

    def listener_callback_armors(self, msg):
        try:
            dt = time_diff()
            self.kf_cx.dt = dt
            self.kf_cy.dt = dt
            self.kf_h.dt = dt
            # 将JSON格式的数据转换回Python字典
            armors_dict = json.loads(msg.data)

            if self.tracking_color == 0: 
                color_str = '红色'
                self.get_logger().info(f"得到需要 追踪 的 {color_str} 装甲板 {self.tracking_armor}")
            elif self.tracking_color == 1: 
                color_str = "蓝色"
                self.get_logger().info(f"得到需要 追踪 的 {color_str} 装甲板 {self.tracking_armor}")
            elif self.tracking_color == -1: 
                color_str = "未知"
                self.get_logger().info(f"不需要 追踪 装甲板 ")
            else :
                color_str = "未知"
                self.get_logger().info(f"颜色格式错误")

            # 选择要跟踪的装甲板
            self.tracking_armor = select_tracking_armor(armors_dict, self.tracking_color, self.reflection_hight_tol, self.reflection_cx_tol)  # 0表示红色
            
            if not self.tracking_armor:  # 检查 tracking_armor 是否为空
                logger.info("tracking_armor is empty, returning default values.")
                if self.use_kf == True :
                    self.lost += 1
                    if self.lost <= self.frame_add:
                        self.kf_cx.predict()  # 进行预测
                        self.kf_cy.predict()
                        self.kf_h.predict()
                        self.center_last = (self.kf_cx.get_state(), self.kf_cy.get_state())  # 获取预测的状态
                        self.height_last = self.kf_h.get_state()
                    else :
                        self.center_last = (0, 0) 
                        self.height_last = 0
                else :
                    self.center_last = (0, 0) 
                    self.height_last = 0

            else:
                self.center_last = self.tracking_armor["center"]
                self.height_last = self.tracking_armor["height"]
                if self.use_kf == True :
                    self.lost = 0
                    self.kf_cx.predict()  # 进行预测
                    self.kf_cy.predict()
                    self.kf_h.predict()
                    self.kf_cx.update(self.center_last[0])  # 更新状态   
                    self.kf_cy.update(self.center_last[1])  # 更新状态 
                    self.kf_h.update(self.height_last)            
                    self.center_last = (self.kf_cx.get_state(), self.kf_cy.get_state())  # 获取预测的状态
                    self.height_last = self.kf_h.get_state()

                    print(f"预测的 cx: {self.center_last[0]}, cy: {self.center_last[1]}, h: {self.height_last}")

            yaw, pitch, deep = pixel_to_angle_and_deep(self.height_last, self.center_last, 72, self.pic_width) 

            # 将装甲板信息字典转换为msg消息定义的格式
            tracking_armor_json = json.dumps(self.tracking_armor)

            # 创建自定义消息对象并添加Header
            tracking_armor_msg = ArmorTracking()
            tracking_armor_msg.header = Header()  # 创建并设置Header
            tracking_armor_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
            tracking_armor_msg.header.frame_id = 'tracking_armor_frame'  # 可根据需要设置frame_id

            # 设置的装甲板信息
            tracking_armor_msg.data  = tracking_armor_json
            tracking_armor_msg.yaw   = float(yaw + self.offset_yaw)
            tracking_armor_msg.pitch = float(pitch + self.offset_pitch + deep * self.deep_buff)
            tracking_armor_msg.deep  = float(deep)

            self.get_logger().info(f"发布的 tracking_armor_msg: {tracking_armor_msg}")

            # 发布消息
            self.pub_tracker.publish(tracking_armor_msg)
 
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {e}')


    def listener_callback_serial(self, msg):
        # 获取 Decision 数据
        # self.get_logger().info(f'Received Decision data: {msg}')

        # 这里可以对串口数据进行进一步处理
        if self.follow_decision == 1:
            if self.tracking_color != msg.color:
                self.tracking_color = msg.color
                self.get_logger().warn(f'颜色改变为 {self.tracking_color} 号颜色')

def main(args=None):                              # ROS2节点主入口main函数
    rclpy.init(args=args)      # ROS2 Python接口初始化               
    node = ArmorTrackerNode("armor_tracker_node")     # 创建ROS2节点对象
    rclpy.spin(node)                              # 循环等待ROS2退出
    node.destroy_node()                           # 销毁节点对象
    rclpy.shutdown()                              # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
