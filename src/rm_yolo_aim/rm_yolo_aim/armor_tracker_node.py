import json
import time                             # JSON序列化库
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型
from sensor_msgs.msg import Image       # 图像消息类型
from rm_interfaces.msg import ArmorsMsg, ArmorTracking  # 导入自定义消息类型
from rm_yolo_aim.armor_tracker import select_tracking_armor, pixel_to_angle_and_deep
from rcl_interfaces.msg import SetParametersResult  # 导入 SetParametersResult 消息类型
from rm_yolo_aim.Kalman import KalmanFilter
from loguru import logger
import time  # 导入时间模块
#kalmanfilter = KalmanFilter()

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
        self.sub_armors = self.create_subscription(
            ArmorsMsg, '/detector/armors_info', self.listener_callback_armors, 10)  # 订阅装甲板信息
        self.sub_cam    = self.create_subscription(
            Image, 'image_raw', self.listener_callback_cam, 10)

        self.sub_serial = self.create_subscription(
            String, '/uart/receive', self.listener_callback_serial, 10)  # 订阅串口数据
        self.kf = KalmanFilter()
        self.pub_tracker = self.create_publisher(ArmorTracking, '/tracker/target', 10)
        self.center_last = (0, 0)
        self.tracking_color = 1    # 0蓝色表示, 1表示红色, 现初始化为红色
        self.tracking_armor = None
        self.height = 0
        self.use_kf = True
        self.pic_width = 666       # 随便初始化一个图像宽度
        self.lost = 0
        self.start_time = None
        self.time_diff_flag = False

        self.declare_parameter('use_kf', self.use_kf)  # 声明 use_kf 参数
        self.add_on_set_parameters_callback(self.param_callback)  # 添加参数回调
        self.get_logger().info('Armor Tracker Node has started.')

    def param_callback(self, params):
        if params[0].name == 'use_kf':
            self.use_kf = params[0].value
        return SetParametersResult(successful=True)  # 返回成功结果

    def listener_callback_cam(self, data):
        if self.pic_width != data.width:
            self.pic_width = data.width

    def listener_callback_armors(self, msg):
        try:
            self.kf.dt = time_diff()
            print(f"time_dt: {self.kf.dt}")
            # 将JSON格式的数据转换回Python字典
            armors_dict = json.loads(msg.data)
            # self.get_logger().info(f'Received armors data: {armors_dict}')

            # 选择要跟踪的装甲板
            # self.tracking_armor = select_tracking_armor(armors_dict, 0)  # 0表示红色
            self.tracking_armor = select_tracking_armor(armors_dict, self.tracking_color)  # 0表示红色
            self.get_logger().info(f"得到需要 追踪 的装甲板 {self.tracking_armor}")
            
            if not self.tracking_armor:  # 检查 tracking_armor 是否为空
                logger.info("tracking_armor is empty, returning default values.")
                if self.time_diff_flag == False :
                    self.start_time = time.time()               
                    self.time_diff_flag = True
                if self.time_diff_flag == True : 
                    elapsed_time = time.time() - self.start_time  # 计算经过的时间
                    if elapsed_time > 1 :
                        self.center_last = (0, 0)  
                        self.height = 0
                else:
                    if self.use_kf == True :
                        self.kf.predict()  # 进行预测
                        self.center_last = self.kf.get_state()  # 获取预测的状态
                    else :
                        self.center_last = (0, 0) 

            else:
                self.time_diff_flag = False  # 停止计时器
                self.center_last = self.tracking_armor["center"]
                self.height = self.tracking_armor["height"]
                if self.use_kf == True :
                    self.kf.predict()  # 进行预测
                    self.kf.update(self.center_last[0], self.center_last[1])  # 更新状态              
                    self.center_last = self.kf.get_state()  # 获取预测的状态
                    print(f"预测的 yaw: {self.center_last[0]:.2f}, pitch: {self.center_last[1]:.2f}")
            yaw, pitch, deep = pixel_to_angle_and_deep(self.height, self.center_last, 72, self.pic_width) 

            self.get_logger().info(f"yaw, pitch, deep, use_kf: {yaw, pitch, deep, self.use_kf}")
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
            tracking_armor_msg.kf  = self.use_kf

            # 发布消息
            self.pub_tracker.publish(tracking_armor_msg)
 
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {e}')


    def listener_callback_serial(self, msg):
        # 获取串口数据
        serial_data = msg.data
        self.get_logger().info(f'Received serial data: {serial_data}')

        # 这里可以对串口数据进行进一步处理
        if self.tracking_color != msg.tracking_color:
            self.tracking_color = msg.tracking_color

def main(args=None):                              # ROS2节点主入口main函数
    rclpy.init(args=args)      # ROS2 Python接口初始化               
    node = ArmorTrackerNode("armor_tracker_node")     # 创建ROS2节点对象
    rclpy.spin(node)                              # 循环等待ROS2退出
    node.destroy_node()                           # 销毁节点对象
    rclpy.shutdown()                              # 关闭ROS2 Python接口

if __name__ == '__main__':
    main()
