from ultralytics import YOLO
import cv2
from loguru import logger
import serial

class YOLOObjectDetector:
    def __init__(self, model_path, serial_port='/dev/ttyUSB0', baud_rate=115200):
        self.logger = logger
        self.model = YOLO(model_path) # 加载模型
        # self.logger.info(f'加载模型 {self.model}')
        self.com = serial.Serial(serial_port, baud_rate)
        self.target_class = 'R4'

        # 打印所有类别名称和它们的索引
        for idx, name in self.model.names.items():
            self.logger.info(f'类别索引: {idx}, 名称: {name}')

    def get_class_index(self, class_name): # 根据类别名称返回类别索引
        return next((idx for idx, name in self.model.names.items() if name == class_name), None)

    def predict(self, img):
        """
        对输入图像进行预测，寻找并标记目标。

        参数:
        img: 输入的图像，应为numpy数组格式。

        返回:
        annotated_img: 标记后的图像。
        max_center: 最大面积目标的中心坐标，如果未检测到目标则返回None。
        """
        results = self.model(img)
        # self.logger.info(f'用时 {results[0].speed}')
        annotated_img = results[0].plot()
        img_height, img_width = img.shape[:2]
        # self.logger.info(f'图像尺寸 {img_height} x {img_width}')

        # 获取目标类别的索引
        target_idx = self.get_class_index(self.target_class)

        # 如果未找到目标类别，记录警告并返回
        if target_idx is None:
            self.logger.warning(f'未找到类别 "{self.target_class}"')
            return annotated_img, None

        # 获取所有检测到的框
        boxes = results[0].boxes
        
        # 初始化最大面积和对应的中心点
        max_area = 0
        max_center = None

        # 遍历所有框，寻找最大面积的目标
        for box in boxes:
            # 如果框的类别是目标类别
            if box.cls == target_idx:
                # 计算框的坐标和面积
                x1, y1, x2, y2 = box.xyxy[0]
                area = (x2 - x1) * (y2 - y1)
                
                # 如果当前面积大于最大面积，更新最大面积和中心点
                if area > max_area:
                    max_area = area
                    max_center = ((x1 + x2) / 2, (y1 + y2) / 2)

        # 如果找到了目标，将其中心点转换为像素坐标，并计算相对于图像中心的偏移量
        if max_center is not None:
            pixel_x = int(max_center[0].item())
            pixel_y = int(max_center[1].item())

            # 记录目标中心的像素坐标
            # self.logger.info(f"pixel_x: {pixel_x}, pixel_y: {pixel_y}")

            servo_x = pixel_x - int(img_width  / 2)
            servo_y = pixel_y - int(img_height / 2)

            # 创建要发送的串口消息
            uart_msg = f'[{servo_x},{servo_y}]'

            # 将消息发送到串口
            self.com.write(uart_msg.encode('ascii'))
            # 记录发送的串口消息
            self.logger.info(f'发送到串口的数据: {uart_msg}')

            # 在图像上标记目标中心和准星
            cv2.circle(annotated_img, (int(pixel_x), int(pixel_y)), 5, (0, 0, 255), -1)  # 目标中心点
            cv2.circle(annotated_img, (int(img_width / 2), int(img_height / 2)), 2, (0, 255, 0), -1)# 准星
                
            # 记录最大面积目标的中心坐标
            # self.logger.info(f'最大面积目标的中心坐标: {max_center}')
        else:
            # 如果未检测到目标，记录信息
            self.logger.info('未检测到目标')

        # 返回标记后的图像和最大目标的中心坐标
        return annotated_img, max_center

    @staticmethod
    def find_available_cameras(start_index=0, end_index=9):
        available_cameras = []
        for index in range(start_index, end_index + 1):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                available_cameras.append(index)
                cap.release()
        return available_cameras

    def start_detection(self, camera_index=2):
        self.logger.info(f'可用摄像头：{self.find_available_cameras()}')
        self.logger.info('开始预测')

        cap = cv2.VideoCapture(camera_index)

        if cap is not None:
            while True:
                ret, frame = cap.read()
                if ret:
                    PD_frame, max_center = self.predict(frame)

                    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
                    cv2.imshow("frame", PD_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        cv2.destroyAllWindows()
        if cap is not None:
            cap.release()

if __name__ == '__main__':
    detector = YOLOObjectDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')
    detector.start_detection()