from ultralytics import YOLO
import cv2
import os
import numpy as np
from loguru import logger

USER = os.environ['USER']

class ArmorDetector:
    def __init__(self):
        model_path = (
        f"/home/{USER}/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best_openvino_model/"
    )
        self.logger = logger
        self.model = YOLO(model_path)  # 加载模型
        
        # self.logger.info(f'加载模型 {self.model}')
        # self.com = serial.Serial(serial_port, baud_rate)

    def calculate_perimeter(self, bbox):

        x1, y1, x2, y2 = bbox[0]
        width = x2 - x1
        height = y2 - y1
        perimeter = int(2 * (width + height))

        return perimeter    
    def calculate_height(self, bbox):

        x1, y1, x2, y2 = bbox[0]
        height = int(y2 - y1)

        return height

    def calculate_center(self, bbox, img):
        x1, y1, x2, y2 = bbox[0]
        height, width = img.shape[:2]

        center_x = int((x1 + x2) / 2 - (width / 2))
        center_y = -int((y1 + y2) / 2 - (height / 2)) # 图片的y轴和准星的y轴是反的

        return center_x, center_y
    
    def undistort_image(self, cv_image, camera_info):
        if camera_info is None:
            self.get_logger().warn('Camera info not received yet.')
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

    def detect_armor(self, img):
        results = self.model(img)
        img = results[0].plot()  # 获取绘制后的图像

        armors = {}

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)  # 获取类别ID
                # confidence = float(box.conf)  # 获取置信度
                boxx = box.xyxy.tolist()  # 获取边界框坐标

                height = self.calculate_height(boxx)
                center_x, center_y = self.calculate_center(boxx, img)

                armors[str(center_x)] = {
                    "class_id": class_id,
                    # "confidence": confidence, # 置信度
                    # "bbox": boxx, # 边界框坐标
                    "height": height,
                    "center": [center_x, center_y],
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
        #cap = cv2.VideoCapture("/home/{USER}/ros_ws/src/rm_yolo_aim/rm_yolo_aim/test.mp4")

        if cap is not None:
            while True:
                ret, frame = cap.read()
                if ret:
                    PD_frame, max_center = self.detect_armor(frame)

                    logger.info(f"预测结果：\n{max_center}")

                    if 'DISPLAY' in os.environ:  # 判断是否为图形界面环境
                        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
                        cv2.imshow("frame", PD_frame)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break

        cv2.destroyAllWindows()
        if cap is not None:
            cap.release()


if __name__ == "__main__":

    detector = ArmorDetector()
    detector.logger.info("开始预测")
    detector.start_detection()

    {
        "179": {"class_id": 7, "size": 290, "center": [179, 35]},
        "-143": {"class_id": 3, "size": 288, "center": [-143, -35]},
        "149": {"class_id": 3, "size": 191, "center": [149, 36]},
        "-113": {"class_id": 2, "size": 300, "center": [-113, -35]},
    }
