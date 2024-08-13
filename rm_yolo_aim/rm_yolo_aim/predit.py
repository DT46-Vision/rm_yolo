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
        self.target_class = 'B1'

        # 打印所有类别名称和它们的索引
        for idx, name in self.model.names.items():
            self.logger.info(f'类别索引: {idx}, 名称: {name}')

    def get_class_index(self, class_name): # 根据类别名称返回类别索引
        return next((idx for idx, name in self.model.names.items() if name == class_name), None)

    def predict(self, img):
        results = self.model(img)
        # self.logger.info(f'用时 {results[0].speed}')
        annotated_img = results[0].plot()
        img_height, img_width = img.shape[:2]
        # self.logger.info(f'图像尺寸 {img_height} x {img_width}')

        target_idx = self.get_class_index(self.target_class)

        if target_idx is None:
            self.logger.warning(f'未找到类别 "{self.target_class}"')
            return annotated_img, None

        boxes = results[0].boxes
        max_area = 0
        max_center = None

        for box in boxes:
            if box.cls == target_idx:
                x1, y1, x2, y2 = box.xyxy[0]
                area = (x2 - x1) * (y2 - y1)

                if area > max_area:
                    max_area = area
                    max_center = ((x1 + x2) / 2, (y1 + y2) / 2)

        if max_center is not None:
            pixel_x = int(max_center[0].item())
            pixel_y = int(max_center[1].item())

            # self.logger.info(f"pixel_x: {pixel_x}, pixel_y: {pixel_y}")

            servo_x = pixel_x - int(img_width  / 2)
            servo_y = pixel_y - int(img_height / 2)

            uart_msg = f'[{servo_x},{servo_y}]'

            self.com.write(uart_msg.encode('ascii'))
            self.logger.info(f'发送到串口的数据: {uart_msg}')

            cv2.circle(annotated_img, (int(pixel_x), int(pixel_y)), 5, (0, 0, 255), -1)  # 目标中心点
            cv2.circle(annotated_img, (int(img_width / 2), int(img_height / 2)), 2, (0, 255, 0), -1)# 准星
            
            # self.logger.info(f'最大面积目标的中心坐标: {max_center}')
        else:
            self.logger.info('未检测到目标')

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
    detector = YOLOObjectDetector('/home/dbink/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')
    detector.start_detection()