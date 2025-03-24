import cv2  # 导入 OpenCV 库
import numpy as np  # 导入 NumPy 库
import math  # 导入数学库
from loguru import logger
import time

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


def calculate_distance(point1, point2):
    x1, y1 = point1  # 解包第一个点的坐标
    x2, y2 = point2  # 解包第二个点的坐标
    # 使用距离公式计算并返回结果
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def adjust(w_h, angle):  # 调整矩形的函数
    (w, h) = w_h
    if w > h:  # 如果宽度大于高度
        w, h = h, w  # 交换宽度和高度
        if angle >= 0 :
            angle = angle - 90 # 调整角度，使其跟随高度
        elif angle < 0 :
            angle = angle + 90
    return (w, h), angle # 返回调整后的结果


def angle_to_slope(angle_degrees):
    angle_radians = math.radians(angle_degrees)    # 将角度转换为弧度
    slope = math.tan(angle_radians)    # 计算斜率
    return slope


def project(polygon, axis):  # 投影计算
    return np.dot(polygon, axis).min(), np.dot(polygon, axis).max()  # 计算最小值和最大值


def is_coincide(a, b):  # 检查两个多边形是否重叠
    for polygon in (a, b):  # 遍历多边形 a 和 b
        for i in range(len(polygon)):  # 遍历每个多边形的边
            p1, p2 = polygon[i], polygon[(i + 1) % len(polygon)]  # 获取边的两个端点
            normal = (p2[1] - p1[1], p1[0] - p2[0])  # 计算法向量
            min_a, max_a = project(a, normal)  # 计算多边形 a 的投影的最小值和最大值
            min_b, max_b = project(b, normal)  # 计算多边形 b 的投影的最小值和最大值
            if max_a < min_b or max_b < min_a:  # 检查是否相交
                return False  # 不相交则返回 False


class Light:  # 定义灯条类
    def __init__(self, up, down, angle, color):  # 初始化灯条的矩形和颜色
        self.cx = int(abs(up[0] - down[0]) / 2 + min(up[0], down[0]))
        self.cy = int(abs(up[1] - down[1]) / 2 + min(up[1], down[1]))
        self.height = calculate_distance(up, down)
        self.color = color  # 设置灯条颜色
        self.up = up
        self.down = down
        self.angle = angle


class Armor:  # 定义装甲板类
    def __init__(self, light1, light2, height, type):  # 初始化装甲板的矩形
        armor_cx = int(abs(light1.cx - light2.cx) / 2 + min(light1.cx, light2.cx))
        armor_cy = int(abs(light1.cy - light2.cy) / 2 + min(light1.cy, light2.cy))
        self.center = (armor_cx, armor_cy)
        self.light1_up = light1.up
        self.light1_down = light1.down
        self.light2_up = light2.up
        self.light2_down = light2.down
        self.color = light1.color  # 装甲板颜色初始化为 None
        self.height = height
        self.type = type # 1 :大装甲板, 0 :小装甲板
    
    def type_class(self):
        if self.color == 0:
            if self.type == 0:
                return 7
            if self.type == 1:
                return 6
            
        if self.color == 1:
            if self.type == 0:
                return 1
            if self.type == 1:
                return 0


class ArmorDetector:  # 定义检测器类
    def __init__(self, detect_color, display_mode, binary_val, light_params, color_params):  # 初始化检测器
        self.img = None
        self.img_binary = None
        self.img_draw = None
        self.lights = []  # 存储灯条列表
        self.armors = []  # 存储装甲板列表
        self.armors_dict = {}  # 存储装甲板信息的字典
        self.binary_val = binary_val  # 二值化阈值
        self.color = detect_color  # 颜色模式
        self.display_mode = display_mode # 显示模式
        self.light_params = light_params  # 灯条参数
        self.armor_color = color_params["armor_color"]  # 装甲板颜色映射
        self.light_color = color_params["light_color"]  # 灯条颜色映射
        self.light_dot = color_params["light_dot"]  # 灯条中心点颜色映射

        
    def process(self, img):  # 处理图像的函数
        self.img = img
        _, self.img_binary = cv2.threshold(cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY), self.binary_val, 255, cv2.THRESH_BINARY)  # 二值化处理
        return self.img_binary
    

    def find_lights(self, img_binary):  # 查找灯条的函数
        lights = []
        is_lights = []
        is_lights_flited = []
        contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 查找轮廓
        
        for contour in contours:
            if cv2.contourArea(contour) >= self.light_params["light_area_min"]:
                center, w_h, angle = cv2.minAreaRect(contour)
                w_h, angle = adjust(w_h, angle)
                if angle >= self.light_params["light_angle_min"] and angle <= self.light_params["light_angle_max"]:
                    rect = center, w_h, angle
                    is_lights.append(rect)
        
        for is_light in is_lights:  # 遍历所有光源
            is_overlapping = False  # 标记当前光源是否与其他光源重叠
            for other_is_light in is_lights:  # 遍历所有光源
                if is_light != other_is_light:  # 确保不是同一个光源
                    # 获取其他光源的边界框点并转换为整数# 检查是否重叠# 获取当前光源的边界框点并转换为整数
                    if is_coincide(cv2.boxPoints(is_light).astype(int), cv2.boxPoints(other_is_light).astype(int)):  
                        is_overlapping = True  # 设置为重叠标记
                        break  # 找到重叠后跳出内层循环
            if not is_overlapping:  # 如果没有重叠
                is_lights_flited.append(is_light)  # 添加到不重叠的灯条列表中

        for rect in is_lights_flited:  # 遍历过滤后的灯条
            box = cv2.boxPoints(rect).astype(int)  # 获取旋转矩形的四个点

            right_up_x, right_up_y = box[0]       # 通过角点裁剪出对应的区域
            left_up_x, left_up_y = box[3]

            up_x = int(abs(right_up_x - left_up_x) / 2 + min(right_up_x, left_up_x))
            up_y = int(abs(right_up_y - left_up_y) / 2 + min(right_up_y, left_up_y))
            up = (up_x, up_y)

            right_down_x, right_down_y = box[1]
            left_down_x, left_down_y = box[2]
            
            down_x = int((abs(right_down_x - left_down_x) / 2 + min(right_down_x, left_down_x)))
            down_y = int((abs(right_down_y - left_down_y) / 2 + min(right_down_y, left_down_y)))
            down = (down_x, down_y)

            length = int(np.sqrt((down_x - up_x) ** 2 + (down_y - up_y) ** 2))            # 计算线段的长度

            roi = np.zeros((1, length, 3), dtype=np.uint8)            # 创建一个新图像以存储裁剪的线段像素

            for i in range(length):            # 计算线段上的每个像素
                t = i / length  # 计算比例
                current_x = int(up_x + (down_x - up_x) * t)
                current_y = int(up_y + (down_y - up_y) * t)
                            
                if 0 <= current_x < self.img.shape[1] and 0 <= current_y < self.img.shape[0]:                # 添加边界检查
                    roi[0, i] = self.img[current_y, current_x]  # 保存像素值

            sum_r, sum_b = np.sum(roi[:, :, 2]), np.sum(roi[:, :, 0])  # 计算红色和蓝色的总和

            if self.color in [1, 2] and sum_b > sum_r * self.light_params["light_blue_ratio"]:  # 根据模式识别颜色
                light_blue = Light(up, down, rect[2], 1)  # 创建蓝色灯条对象
                lights.append(light_blue)  # 添加蓝色灯条

            elif self.color in [0, 2] and sum_r > sum_b * self.light_params["light_red_ratio"]:  # 根据模式识别颜色
                light_red = Light(up, down, rect[2], 0)  # 创建红色灯条对象
                lights.append(light_red)  # 添加红色灯条

        self.lights = lights

        return self.lights


    def is_close(self, light1, light2, light_params):  # 检查两个矩形是否接近则返回一个高度
        if abs(light1.cy - light2.cy) < light_params["cy_tol"]: 
            height = min(light1.height, light2.height)
            distance = calculate_distance((light1.cx, light1.cy), (light2.cx, light2.cy))                    
            if distance > height :
                if distance < height * self.light_params["height_multiplier"]:
                    return 0, height # first small armor
                elif distance < height * 1.86 * self.light_params["height_multiplier"]:
                    return 1, height # last large armor
                
        angle_diff = abs(light1.angle - light2.angle)  # 计算角度差

        if angle_diff <= light_params["light_angle_tol"]:  # 判断角度差是否在容忍范围内
            if abs(light1.height - light2.height) <= light_params["height_tol"]:  # 判断高差
                light1_angle = math.degrees(math.atan2(light1.up[1]- light1.down[1], light1.up[0] - light1.down[0]))  # 计算连线角度
                light2_angle = math.degrees(math.atan2(light2.up[1]- light2.down[1], light2.up[0] - light2.down[0]))  # 计算连线角度                
                line_angle = math.degrees(math.atan2(light1.cy - light2.cy, light1.cx - light2.cx))  # 计算连线角度

                slope1 = angle_to_slope(light1_angle)                # 计算斜率
                slope2 = angle_to_slope(light2_angle)           
                slope_line = angle_to_slope(line_angle)
                
                if abs(slope1 * slope_line + 1) < light_params["vertical_discretization"] or abs(slope2 * slope_line + 1) < light_params["vertical_discretization"]:
                    height = max(light1.height, light2.height)
                    distance = calculate_distance((light1.cx, light1.cy), (light2.cx, light2.cy))
                    if distance > height :
                        if distance < height * self.light_params["height_multiplier"]:
                            return 0, height # first small armor
                        elif distance < height * 1.86 * self.light_params["height_multiplier"]:
                            return 1, height # last large armor
                        
        return None, None # 不满足条件则返回 None


    def is_armor(self, lights):  # 检查是否为装甲板的函数
        armors = []
        processed_indices = set()  # 用于存储已处理的矩形索引
        lights_count = len(lights)  # 存储列表长度，避免重复计算

        for i in range(lights_count):  # 遍历所有灯条
            if i in processed_indices:  # 如果该矩形已处理，跳过
                continue

            light = lights[i]  # 取出当前灯条

            for j in range(lights_count) : 
                if j != i and j not in processed_indices and lights[j].color == light.color :  # 如果找到接近的灯条
                    type, height = self.is_close(light, lights[j], self.light_params)
                    if height is not None :
                        armor = Armor(light, lights[j], height, type)  # 创建装甲板对象
                        armors.append(armor)  # 添加装甲板到列表
                        processed_indices.update([i] + [j])  # 将已处理的矩形索引添加到 processed_indices 中
        self.armors = armors
        return self.armors

    def id_armor(self):  # 为装甲板分配 ID 的函数
        armors_dict = {}
        img_height, img_width = self.img.shape[:2]

        for armor in self.armors:  # 遍历所有装甲板矩形
            center = armor.center

            center_x = int(center[0] - (img_width / 2))
            center_y = -int(center[1] - (img_height / 2)) # 图片的y轴和准星的y轴是反的

            armors_dict[int(center[0])] = {  # 添加装甲板信息到字典
                "class_id": armor.type_class(),  # 添加 armor_id
                "height": armor.height,  # 添加高度
                "center": [center_x, center_y]  # 添加中心点
            }
        self.armors_dict = armors_dict
        return armors_dict
    

    def find_armor(self):  # 查找装甲板的函数
        self.is_armor(self.lights)  # 查找装甲板
        armors_dict = self.id_armor()  # 为装甲板分配 ID
        return armors_dict


    def draw_lights(self, img):  # 绘制灯条的函数
            for light in self.lights:  # 遍历灯条
                # 绘制直线
                cv2.line(img, light.up, light.down, self.light_color[light.color], 1) 
                # 绘制中心点
                cv2.circle(img, (light.cx, light.cy), 1, self.light_dot[light.color], -1)  # 5是半径，-1表示填充
            return img
    

    def draw_armors(self, img):  # 绘制装甲板的函数
        for armor in self.armors:  # 遍历装甲板
            center = armor.center
            img_height, img_width = self.img.shape[:2]
            center_x = int(center[0] - (img_width / 2))
            center_y = -int(center[1] - (img_height / 2)) # 图片的y轴和准星的y轴是反的
            cv2.line(img, armor.light1_up, armor.light2_down, self.armor_color[armor.color], 1) 
            cv2.line(img, armor.light2_up, armor.light1_down, self.armor_color[armor.color], 1) 
            cv2.putText(img, f"({center_x}, {center_y})", (center[0], center[1]),  # 在图像上标记坐标
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 255, 255), 2)  # 绘制文本
        return img


    def draw_img(self):
        self.draw = self.img.copy()
        self.draw = self.draw_armors(self.draw)  # 绘制装甲板
        self.draw = self.draw_lights(self.draw)  # 绘制灯条
        return self.draw


    def undistort_image(self, cv_image, camera_info):
        dt = time_diff()
        print(f"图片矫正last:{dt}")
        if camera_info is None:
            logger.error('Camera info not received yet.')
            return None
        
        K = np.array(camera_info.k).reshape(3, 3)
        D = np.array(camera_info.d)
        height, width = camera_info.height, camera_info.width
        logger.info(f"高:{height}, 宽:{width}")
        
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K, D, (width, height), 1, (width, height))
        
        dt = time_diff()
        print(f"计算新的相机矩阵和ROI:{dt}")
        
        # 预先计算映射表
        map1, map2 = cv2.initUndistortRectifyMap(K, D, None, new_camera_matrix, (width, height), cv2.CV_32FC1)
        
        dt = time_diff()
        print(f"计算映射表:{dt}")
        
        # 使用映射表进行重映射
        undistorted_image = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR)
        
        dt = time_diff()
        print(f"重映射图像:{dt}")
        
        # 裁剪图像
        x, y, w, h = roi
        undistorted_image = undistorted_image[y:y+h, x:x+w]
        
        dt = time_diff()
        print(f"裁剪图像:{dt}")
        
        return undistorted_image


    def display(self):
        if self.display_mode == 1 :
            return self.img_binary, None
        elif self.display_mode == 2 :
            self.draw = self.draw_img() 
            return self.img_binary, self.draw
        elif self.display_mode == 0 :
            return None, None
        else:
            print("Invalid display mode")
            return None, None


    def detect_armor(self, frame = None):  # 检测函数
        if frame is not None :
            frame_binary = self.process(frame)  # 处理图像
            self.find_lights(frame_binary)  # 查找灯条
            armors_dict = self.find_armor()  # 查找装甲板
            return armors_dict
        else :
            return self.armors_dict
        

if __name__ == "__main__":  # 主程序入口
    # 模式参数字典
    detect_color =  2  # 颜色参数 0: 识别红色装甲板, 1: 识别蓝色装甲板, 2: 识别全部装甲板
    display_mode = 2 # 显示模式 0: 不显示, 1: 显示二值化图, 2: 显示二值化图和结果图像
    # 图像参数字典
    binary_val = 225
    light_params = {
        "light_area_min": 5,  # 最小灯条面积
        "light_angle_min": -35,  # 最小灯条角度
        "light_angle_max": 35,  # 最大灯条角度
        "light_red_ratio": 1,
        "light_blue_ratio": 1,
        "light_angle_tol": 7,  # 灯条角度容差
        "vertical_discretization": 2.1,  # 垂直离散
        "height_tol": 18,  # 高度容差
        "cy_tol": 5,  # 中心点的y轴容差
        "height_multiplier": 2.7, 
    }
    # 颜色参数字典
    color_params = {
        "armor_color": {1: (255, 255, 0), 0: (128, 0, 128)},  # 装甲板颜色映射
        "light_color": {1: (200, 71, 90), 0: (0, 100, 255)},  # 灯条颜色映射
        "light_dot": {1: (0, 0, 255), 0: (255, 0, 0)}  # 灯条中心点颜色映射
    }
    detector = ArmorDetector(detect_color, display_mode, binary_val, light_params, color_params)  # 创建检测器对象
    info = detector.detect_armor(cv2.resize(cv2.imread('src/rm_yolo_aim/test/b.jpg'), (640, 480)))  # 读取图像并进行检测
    bin, img = detector.display()  # 显示图像
    cv2.namedWindow('result', cv2.WINDOW_NORMAL)
    cv2.imshow('bin', bin)
    cv2.imshow('result', img)
    logger.info(info) # 打印检测结果
    cv2.waitKey(0)  # 等待按键
    cv2.destroyAllWindows()  # 关闭所有窗口