from networkx import center
from ultralytics import YOLO
import cv2
import os
from loguru import logger
import serial


class ArmorDetector:
    def __init__(self, model_path, serial_port="/dev/ttyUSB0", baud_rate=115200):
        self.logger = logger
        self.model = YOLO(model_path)  # 加载模型
        # self.logger.info(f'加载模型 {self.model}')
        # self.com = serial.Serial(serial_port, baud_rate)

    def calculate_perimeter(self, bbox):

        x1, y1, x2, y2 = bbox[0]
        width = x2 - x1
        height = y2 - y1
        perimeter = 2 * (width + height)

        return perimeter
    
    def calculate_center(self, bbox, img):
        x1, y1, x2, y2 = bbox[0]
        height, width = img.shape[:2]

        center_x = int((x1 + x2) / 2 - (width  / 2))
        center_y = int((y1 + y2) / 2 - (height / 2))

        return center_x, center_y

    def detect_armor(self, img):
        results = self.model(img)
        img = results[0].plot()  # 获取绘制后的图像

        armors = {}

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)       # 获取类别ID
                confidence = float(box.conf)  # 获取置信度
                boxx = box.xyxy.tolist()      # 获取边界框坐标

                size = self.calculate_perimeter(boxx)
                center_x, center_y = self.calculate_center(boxx, img)

                # 使用 center_x 作为键，装甲信息作为值
                armors[str(center_x)] = {
                    "class_id": class_id,
                    # "confidence": confidence, # 置信度
                    # "bbox": boxx, # 边界框坐标
                    "size": size,
                    "center": [center_x, center_y]
                }

        return img, armors


    @staticmethod
    def find_available_cameras(start_index=0, end_index=9):
        available_cameras = []
        for index in range(start_index, end_index + 1):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                available_cameras.append(index)
                cap.release()
        return available_cameras

    def start_detection(self, camera_index=None):
        if camera_index is None:
            camera_index = self.find_available_cameras()

        self.logger.info(f"可用摄像头：{camera_index}")
        self.logger.info("开始预测")

        cap = cv2.VideoCapture(camera_index[0])
        # cap = cv2.VideoCapture("/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/test.mp4")

        if cap is not None:
            while True:
                ret, frame = cap.read()
                if ret:
                    PD_frame, max_center = self.detect_armor(frame)

                    logger.info(f"预测结果：\n{max_center}")

                    if not os.isatty(0):
                        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
                        cv2.imshow("frame", PD_frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break

        cv2.destroyAllWindows()
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    # OpenVINO模型路径
    ov_model_path = (
        "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best_openvino_model/"
    )
    detector = ArmorDetector(ov_model_path)

    # detector = ArmorDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')
    detector.start_detection()
