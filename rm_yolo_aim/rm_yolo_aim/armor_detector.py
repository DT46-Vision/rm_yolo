from ultralytics import YOLO
import cv2
from loguru import logger
import serial

class ArmorDetector:
    def __init__(self, model_path, serial_port='/dev/ttyUSB0', baud_rate=115200):
        self.logger = logger
        self.model = YOLO(model_path) # 加载模型
        # self.logger.info(f'加载模型 {self.model}')
        #self.com = serial.Serial(serial_port, baud_rate)

    def detect_armor(self, img):

        results = self.model(img)

        img = results[0].plot()  # 获取绘制后的图像

        detections = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls)  # 获取类别ID
                confidence = box.conf    # 获取置信度
                detections.append({
                    'class_id': class_id,
                    'confidence': confidence,
                    'bbox': box.xyxy.tolist()  # 获取边界框坐标
                })
        
        return img, detections

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

        self.logger.info(f'可用摄像头：{camera_index}')
        self.logger.info('开始预测')

        cap = cv2.VideoCapture(camera_index[0])
        # cap = cv2.VideoCapture("/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/test.mp4")

        if cap is not None:
            while True:
                ret, frame = cap.read()
                if ret:
                    PD_frame, max_center = self.detect_armor(frame)

                    logger.info(f'预测结果：{max_center}')

                    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
                    cv2.imshow("frame", PD_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        cv2.destroyAllWindows()
        if cap is not None:
            cap.release()

if __name__ == '__main__':
    # OpenVINO模型路径
    ov_model_path = "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best_openvino_model/"
    detector = ArmorDetector(ov_model_path)

    # detector = ArmorDetector('/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt')
    detector.start_detection()