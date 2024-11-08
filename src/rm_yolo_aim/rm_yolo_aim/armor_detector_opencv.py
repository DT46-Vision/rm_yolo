import cv2  # 导入 OpenCV 库
import numpy as np  # 导入 NumPy 库
import math  # 导入数学库
from loguru import logger

class Light:  # 定义灯条类
    def __init__(self, rect, color):  # 初始化灯条的矩形和颜色
        self.color = color  # 设置灯条颜色
        self.rect = rect  # 设置灯条矩形

class Armor:  # 定义装甲板类
    def __init__(self, rect):  # 初始化装甲板的矩形
        self.color = None  # 装甲板颜色初始化为 None
        self.rect = rect  # 设置装甲板矩形

class ArmorDetector:  # 定义检测器类
    def __init__(self, detect_mode, binary_val, light_params, armor_params, color_params):  # 初始化检测器
        self.img = None
        self.img_binary = None
        self.img_darken = None
        self.img_draw = None
        self.binary_val = binary_val  # 二值化阈值
        self.color = detect_mode  # 颜色模式
        self.light_params = light_params  # 灯条参数
        self.armor_params = armor_params  # 装甲板参数
        self.armor_color = color_params["armor_color"]  # 装甲板颜色映射
        self.armor_id = color_params["armor_id"]  # 装甲板 ID 映射
        self.light_color = color_params["light_color"]  # 灯条颜色映射
        self.light_dot = color_params["light_dot"]  # 灯条中心点颜色映射
        self.lights = []  # 存储灯条列表
        self.armors = []  # 存储装甲板列表
        self.armors_dict = {}  # 存储装甲板信息的字典
        
    def darker(self, img):  # 暗化图像函数
        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 转换为 HSV 颜色空间
        hsv_image[:, :, 2] = hsv_image[:, :, 2] * 0.5  # 将 V 通道乘以 0.5，降低亮度
        darker_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)  # 转换回 BGR 颜色空间
        return darker_image  # 返回暗化后的图像
    
    def process(self, img):  # 处理图像的函数
        self.img = img
        self.img_darken = self.darker(cv2.convertScaleAbs(img, alpha=0.5))  # 调整亮度，降低亮度
        _, self.img_binary = cv2.threshold(cv2.cvtColor(self.img_darken, cv2.COLOR_BGR2GRAY), self.binary_val, 255, cv2.THRESH_BINARY)  # 二值化处理
        return self.img_darken, self.img_binary
    
    def adjust(self, rect):  # 调整矩形的函数
        c, (w, h), angle = rect  # 解包矩形的中心、宽高和角度
        if w > h:  # 如果宽度大于高度
            w, h = h, w  # 交换宽度和高度
            angle = (angle + 90) % 360  # 调整角度
            angle = angle - 360 if angle > 180 else angle - 180 if angle > 90 else angle  # 标准化角度：-90 到 90 度
        return c, (w, h), angle  # 返回调整后的结果

    def project(self, polygon, axis):  # 投影计算
        return np.dot(polygon, axis).min(), np.dot(polygon, axis).max()  # 计算最小值和最大值

    def is_coincide(self, a, b):  # 检查两个多边形是否重叠
        for polygon in (a, b):  # 遍历多边形 a 和 b
            for i in range(len(polygon)):  # 遍历每个多边形的边
                p1, p2 = polygon[i], polygon[(i + 1) % len(polygon)]  # 获取边的两个端点
                normal = (p2[1] - p1[1], p1[0] - p2[0])  # 计算法向量
                min_a, max_a = self.project(a, normal)  # 计算多边形 a 的投影的最小值和最大值
                min_b, max_b = self.project(b, normal)  # 计算多边形 b 的投影的最小值和最大值
                if max_a < min_b or max_b < min_a:  # 检查是否相交
                    return False  # 不相交则返回 False
    
    def find_lights(self, img_darken, img_binary):  # 查找灯条的函数
        lights = []
        contours, _ = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 查找轮廓
        lights_filtered = [  # 过滤灯条
            self.adjust(cv2.minAreaRect(contour)) for contour in contours 
            if cv2.contourArea(contour) > self.light_params["light_area_min"] and self.light_params["light_angle_min"] <= self.adjust(cv2.minAreaRect(contour))[2] <= self.light_params["light_angle_max"]  # 过滤小轮廓和大于-30°到30°的旋转矩形
        ]  # 过滤条件合并为一个列表理解式
        lights_filtered = [  # 进一步过滤重叠的灯条
            light for light in lights_filtered 
            if not any(self.is_coincide(cv2.boxPoints(light).astype(int), cv2.boxPoints(other_light).astype(int)) for other_light in lights_filtered if light != other_light)  # 检查重叠
        ]  
        for rect in lights_filtered:  # 遍历过滤后的灯条
            box = cv2.boxPoints(rect).astype(int)  # 获取旋转矩形的四个点
            mask = np.zeros(img_binary.shape, dtype=np.uint8)  # 创建掩膜
            cv2.drawContours(mask, [box], -1, 255, -1)  # 在掩膜上绘制轮廓
            masked_img = cv2.bitwise_and(img_darken, img_darken, mask=mask)  # 按掩膜提取区域
            sum_r, sum_b = np.sum(masked_img[:, :, 2]), np.sum(masked_img[:, :, 0])  # 计算红色和蓝色的总和
            if self.color in [1, 2] and sum_b > sum_r:  # 根据模式识别颜色
                light_blue = Light(rect, 1)  # 创建蓝色灯条对象
                lights.append(light_blue)  # 添加蓝色灯条
            if self.color in [0, 2] and sum_r > sum_b:  # 根据模式识别颜色
                light_red = Light(rect, 0)  # 创建红色灯条对象
                lights.append(light_red)  # 添加红色灯条
        self.lights = lights
        return self.lights

    def is_close(self, rect1, rect2, light_params):  # 检查两个矩形是否接近
        (cx1, cy1), (w1, h1), angle1 = rect1  # 获取第一个旋转矩形的信息
        (cx2, cy2), (w2, h2), angle2 = rect2  # 获取第二个旋转矩形的信息
        distance = math.sqrt((cx1 - cx2) ** 2 + (cy1 - cy2) ** 2)  # 计算中心点之间的距离
        if distance > light_params["light_distance_min"]:  # 首先判断距离是否大于灯条最小距离
            angle_diff = min(abs(angle1 - angle2), 360 - abs(angle1 - angle2))  # 计算角度差
            if angle_diff <= light_params["light_angle_tol"]:  # 判断角度差是否在容忍范围内
                if abs(h1 - h2) <= light_params["height_tol"] and abs(w1 - w2) <= light_params["width_tol"]:  # 判断高宽差
                    line_angle = math.degrees(math.atan2(cy2 - cy1, cx2 - cx1))  # 计算连线角度
                    if line_angle > 90:  # 将角度标准化到 -90° 到 90° 之间
                        line_angle -= 180  
                    elif line_angle < -90:
                        line_angle += 180   
                    if (abs(line_angle - angle1) <= light_params["line_angle_tol"] or abs(line_angle - angle2) <= light_params["line_angle_tol"] or abs(cy1 - cy2) < light_params["cy_tol"]):  
                        return True  # 检查是否垂直或者判断中心点垂直坐标差         
        return False  # 不满足条件则返回 False

    def is_armor(self, lights):  # 检查是否为装甲板的函数
        armors = []
        processed_indices = set()  # 用于存储已处理的矩形索引
        lights_count = len(lights)  # 存储列表长度，避免重复计算
        for i in range(lights_count):  # 遍历所有灯条
            if i in processed_indices:  # 如果该矩形已处理，跳过
                continue
            light = lights[i]  # 取出当前灯条
            close_lights = [j for j in range(lights_count) if j != i and lights[j].color == light.color and self.is_close(light.rect, lights[j].rect, self.light_params)]  # 找到接近的灯条
            if close_lights:  # 如果找到接近的灯条
                group = [light.rect] + [lights[j].rect for j in close_lights]  # 将当前灯条和接近的灯条组合成一组
                points = np.concatenate([cv2.boxPoints(light) for light in group])  # 获取所有灯条的四个顶点
                armor_raw = cv2.minAreaRect(points)  # 计算最小外接矩形
                if self.armor_params["armor_area_min"] <= armor_raw[1][0] * armor_raw[1][1] <= self.armor_params["armor_area_max"]:  # 限制识别到的装甲板面积大小
                    armor_flit = self.adjust(armor_raw)  # 调整装甲板矩形
                    if self.armor_params["armor_height/width_min"] <= armor_flit[1][1] / armor_flit[1][0] <= self.armor_params["armor_height/width_max"]:  # 限制识别到的装甲板矩形高宽比
                        armor = Armor(self.adjust(armor_flit))  # 创建装甲板对象
                        armor.color = light.color  # 设置装甲板颜色
                        armors.append(armor)  # 添加装甲板到列表
                        processed_indices.update([i] + close_lights)  # 将已处理的矩形索引添加到 processed_indices 中
        self.armors = armors
        return self.armors

    def id_armor(self):  # 为装甲板分配 ID 的函数
        armors_dict = {}
        for armor in self.armors:  # 遍历所有装甲板矩形
            center, (width, height), angle = armor.rect  # 获取装甲板矩形的中心、宽高和角度
            max_size = max(width, height)  # 计算最大尺寸
            armors_dict[int(center[0])] = {  # 添加装甲板信息到字典
                "class_id": self.armor_id[armor.color],  # 添加 armor_id
                "height": int(max_size),  # 添加高度
                "center": [int(center[0]), int(center[1])]  # 添加中心点
            }
        self.armors_dict = armors_dict
        return self.armors_dict
    
    def find_armor(self):  # 查找装甲板的函数
        self.is_armor(self.lights)  # 查找装甲板
        self.id_armor()  # 为装甲板分配 ID

    def draw_lights(self, img):  # 绘制灯条的函数
        for light in self.lights:  # 遍历灯条
            box = cv2.boxPoints(light.rect).astype(int)  # 获取灯条的轮廓点
            cv2.drawContours(img, [box], 0, self.light_color[light.color], 1)  # 绘制灯条的轮廓
            cv2.circle(img, tuple(map(int, light.rect[0])), 1, self.light_dot[light.color], -1)  # 绘制灯条的中心点
        return img

    def draw_armors(self, img):  # 绘制装甲板的函数
        for armor in self.armors:  # 遍历装甲板
            center, (max_size, max_size), angle = armor.rect  # 获取装甲板的四个顶点
            box = cv2.boxPoints(((center[0], center[1]), (max_size, max_size), angle)).astype(int)  # 获取装甲板的四个顶点
            cv2.drawContours(img, [box], 0, self.armor_color[armor.color], 2)  # 绘制装甲板的轮廓
            cv2.circle(img, (int(center[0]), int(center[1])), 5, self.armor_color[armor.color], -1)  # 绘制装甲板中心点
            center_x, center_y = map(int, armor.rect[0])  # 获取中心坐标
            cv2.putText(img, f"({center_x}, {center_y})", (center_x, center_y),  # 在图像上标记坐标
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 255, 255), 2)  # 绘制文本
        return img

    def draw_img(self):
        self.draw = self.img.copy()
        self.draw = self.draw_armors(self.draw)  # 绘制装甲板
        self.draw = self.draw_lights(self.draw)  # 绘制灯条
        return self.draw

    def undistort_image(self, cv_image, camera_info):
        if camera_info is None:
            logger.error('Camera info not received yet.')
            return None
        # 获取相机内参和畸变系数
        K = np.array(camera_info.k).reshape(3, 3)
        D = np.array(camera_info.d)
        height, width = camera_info.height, camera_info.width
        logger.info(f"高:{height}, 宽:{width}")
        # 计算新的相机矩阵和ROI
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(K, D, (width, height), 1, (width, height))
        # 矫正图像
        undistorted_image = cv2.undistort(cv_image, K, D, None, new_camera_matrix)
        # 裁剪图像
        x, y, w, h = roi
        undistorted_image = undistorted_image[y:y+h, x:x+w]
        return undistorted_image


    def display(self):  # 显示图像的函数
        cv2.namedWindow("Binary",cv2.WINDOW_NORMAL)       
        cv2.imshow("Binary", self.img_binary)  # 显示二值化图像
        drawn = self.draw_img()
        cv2.namedWindow("Detected",cv2.WINDOW_NORMAL)
        cv2.imshow("Detected", drawn)      # cv2.namedWindow("raw",cv2.WINDOW_NORMAL) # cv2.imshow("raw", self.img)       
        
    def detect_armor(self, frame):  # 检测函数
        frame_darken, frame_binary = self.process(frame)  # 处理图像
        self.find_lights(frame_darken, frame_binary)  # 查找灯条
        self.find_armor()  # 查找装甲板
        self.draw = self.draw_img()    #print(self.armors_dict)  # 打印装甲板信息字典
        return self.draw, self.armors_dict
        
if __name__ == "__main__":  # 主程序入口
    # 模式参数字典
    detect_mode =  2  # 颜色参数 0: 识别红色装甲板, 1: 识别蓝色装甲板, 2: 识别全部装甲板
    # 图像参数字典
    binary_val = 35  
    # 灯条参数字典
    light_params = {
        "light_distance_min": 20,  # 最小灯条距离
        "light_area_min": 5,  # 最小灯条面积
        "light_angle_min": -35,  # 最小灯条角度
        "light_angle_max": 35,  # 最大灯条角度
        "light_angle_tol": 5,  # 灯条角度容差
        "line_angle_tol": 7,  # 线角度容差
        "height_tol": 15,  # 高度容差
        "width_tol": 15,  # 宽度容差
        "cy_tol": 5  # 中心点的y轴容差
    }
    # 装甲板参数字典
    armor_params = {
        "armor_height/width_max": 3.5,  # 装甲板高度与宽度最大比值
        "armor_height/width_min": 1,  # 装甲板高度与宽度最小比值
        "armor_area_max": 11000,  # 装甲板最大面积
        "armor_area_min": 200  # 装甲板最小面积
    }
    # 颜色参数字典
    color_params = {
        "armor_color": {1: (255, 255, 0), 0: (128, 0, 128)},  # 装甲板颜色映射
        "armor_id": {1: 1, 0: 7},  # 装甲板 ID 映射
        "light_color": {1: (200, 71, 90), 0: (0, 100, 255)},  # 灯条颜色映射
        "light_dot": {1: (0, 0, 255), 0: (255, 0, 0)}  # 灯条中心点颜色映射
    }
    detector = ArmorDetector(detect_mode, binary_val, light_params, armor_params, color_params)  # 创建检测器对象
    info = detector.detect_armor(cv2.imread('./photo/red_2.jpg'))  # 读取图像并进行检测
    logger.info(info) # 打印检测结果
    detector.display()  # 显示图像
    cv2.waitKey(0)  # 等待按键
    cv2.destroyAllWindows()  # 关闭所有窗口