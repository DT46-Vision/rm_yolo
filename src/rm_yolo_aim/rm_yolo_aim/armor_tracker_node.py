import json                             # JSON序列化库
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from std_msgs.msg import String, Header # 字符串消息类型和头部消息类型
from sensor_msgs.msg import Image       # 图像消息类型
from rm_interfaces.msg import ArmorsMsg, ArmorTracking  # 导入自定义消息类型
from rm_yolo_aim.armor_tracker import select_tracking_armor, pixel_to_angle_and_deep
import cv2
import numpy as np
global yaw_last, pitch_last
yaw_last = None
pitch_last = None

class KalmanFilter(object):

    def __init__(self):

        self.kalman = cv2.KalmanFilter(4, 2) # 4：状态数，包括（x，y，dx，dy）坐标及速度（每次移动的距离）；2：观测量，能看到的是坐标值
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) # 系统测量矩阵
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]],      np.float32) # 状态转移矩阵
        self.kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)*0.03 # 系统过程噪声协方差
        self.current_measurement = np.array((2, 1), np.float32)
        self.last_measurement = np.array((2, 1), np.float32)
        self.current_prediction = np.zeros((2, 1), np.float32)
        self.last_prediction = np.zeros((2, 1), np.float32)
        self.error_frame = 0

    def track(self,x,y):
        self.last_prediction = self.current_prediction # 把当前预测存储为上一次预测
        self.last_measurement = self.current_measurement # 把当前测量存储为上一次测量

        if abs(self.last_measurement[0] - x) > 64 or abs(self.last_measurement[1] - y) > 48: 
                self.error_frame = self.error_frame + 1
        else :
                pass

        if x ==0 and y == 0 : 
                self.error_frame = self.error_frame + 1
        else :
                pass

        if self.error_frame < 5 and self.error_frame > 0  :
            self.current_measurement = np.array([[np.float32(self.last_prediction[0])], [np.float32(self.last_prediction[1])]])

        else:
            self.current_measurement = np.array([[np.float32(x)], [np.float32(y)]]) # 当前测量
            self.error_frame = 0

        print("error:",self.error_frame)
        self.kalman.correct(self.current_measurement) # 用当前测量来校正卡尔曼滤波器
        self.current_prediction = self.kalman.predict() # 计算卡尔曼预测值，作为当前预测

        lmx, lmy = self.last_measurement[0], self.last_measurement[1] # 上一次测量坐标
        cmx, cmy = self.current_measurement[0], self.current_measurement[1] # 当前测量坐标
        lpx, lpy = self.last_prediction[0], self.last_prediction[1] # 上一次预测坐标
        cpx, cpy = self.current_prediction[0], self.current_prediction[1] # 当前预测坐标

       
        return cpx,cpy


class ArmorTrackerNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2节点父类初始化

        self.sub_armors = self.create_subscription(
            ArmorsMsg, '/detector/armors_info', self.listener_callback_armors, 10)  # 订阅装甲板信息
        
        self.sub_cam    = self.create_subscription(
            Image, 'image_raw', self.listener_callback_cam, 10)

        self.sub_serial = self.create_subscription(
            String, '/uart/receive', self.listener_callback_serial, 10)  # 订阅串口数据
        
        self.pub_tracker = self.create_publisher(ArmorTracking, '/tracker/target', 10)
        
        self.tracking_color = 1    # 0蓝色表示, 1表示红色, 现初始化为红色
        self.tracking_armor = None
        self.pic_width = 666       # 随便初始化一个图像宽度
        self.kalmanfilter = KalmanFilter()


    def listener_callback_cam(self, data):
        if self.pic_width != data.width:
            self.pic_width = data.width

    def listener_callback_armors(self, msg):
        try:
            # 将JSON格式的数据转换回Python字典
            armors_dict = json.loads(msg.data)
            # self.get_logger().info(f'Received armors data: {armors_dict}')

            # 选择要跟踪的装甲板
            # self.tracking_armor = select_tracking_armor(armors_dict, 0)  # 0表示红色
            self.tracking_armor = select_tracking_armor(armors_dict, self.tracking_color)  # 0表示红色
            self.get_logger().info(f"得到需要 追踪 的装甲板 {self.tracking_armor}")

            yaw, pitch, deep = pixel_to_angle_and_deep(self.tracking_armor, 72, self.pic_width) 
            print("yaw,????? pitch:", yaw, pitch)
            yaw_predict, pitch_predict = self.kalmanfilter.track(yaw, pitch)
            self.get_logger().info(f"yaw, pitch, deep: {yaw_predict, pitch_predict, deep}")
            
            # 将装甲板信息字典转换为msg消息定义的格式
            tracking_armor_json = json.dumps(self.tracking_armor)

            # 创建自定义消息对象并添加Header
            tracking_armor_msg = ArmorTracking()
            tracking_armor_msg.header = Header()  # 创建并设置Header
            tracking_armor_msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
            tracking_armor_msg.header.frame_id = 'tracking_armor_frame'  # 可根据需要设置frame_id

            # 设置的装甲板信息
            tracking_armor_msg.data  = tracking_armor_json
            tracking_armor_msg.yaw   = float(yaw_predict)
            tracking_armor_msg.pitch = float(pitch_predict)
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
