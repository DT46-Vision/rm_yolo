from ultralytics import YOLO
import cv2
from loguru import logger

import serial

com = serial.Serial('/dev/ttyUSB0', 115200) 

com.write(f'0,0,[{90},{90}]'.encode('ascii'))

# filter = AdaptiveFilter(0)

logger.info('加载模型...')

# 加载训练后的模型
model = YOLO('models/best.pt')
# model = YOLO('models/yolov8n.pt')

# 打印所有类别名称和它们的索引
for idx, name in model.names.items():
    logger.info(f'类别索引: {idx}, 名称: {name}')

def get_class_index(class_name):
    """根据类别名称返回类别索引"""
    return next((idx for idx, name in model.names.items() if name == class_name), None)

def capture_image(cam=0):
    cap = cv2.VideoCapture(cam)
    ret, img = cap.read()
    logger.info(f'Captured image {ret}')
    cv2.imwrite('output/captured_image.jpg', img)
    cap.release()
    return img


def predict(img, target_class='person'):
    results = model(img)
    logger.info(f'用时 {results[0].speed}')
    annotated_img = results[0].plot()
    # 获取图像尺寸
    img_height, img_width = img.shape[:2]
    logger.info(f'图像尺寸 {img_height} x {img_width}')

    # 获取目标类别的索引
    target_idx = get_class_index(target_class)

    if target_idx is None:
        logger.warning(f'未找到类别 "{target_class}"')
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

        logger.info(f"pixel_x: {pixel_x}, pixel_y: {pixel_y}")

        servo_x = ((pixel_x/img_width)-0.5) * (75/2) * 0.3
        servo_y = ((pixel_y/img_height)-0.5) * (75/2) * 0.1

        com.write(f'[{servo_x},{0},0,0]'.encode('ascii'))
        logger.info(f'发送到串口的数据: {servo_x}, {servo_y}')

        cv2.circle(annotated_img, (int(pixel_x),int(pixel_y)), 5, (0, 0, 255), -1)
        logger.info(f'最大面积目标的 中心坐标: {max_center}')
    else:
        logger.info('未检测到 目标')

    return annotated_img, max_center


def find_available_cameras(start_index=0, end_index=9):
    available_cameras = []
    for index in range(start_index, end_index + 1):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            available_cameras.append(index)
            cap.release()
    return available_cameras


if __name__ == '__main__':
    logger.info(f'可用摄像头：{find_available_cameras()}')

    logger.info('开始预测')

    cap = cv2.VideoCapture(2)

    if cap is not None:
        while True:
            ret, frame = cap.read()
            if ret:
                #PD_frame, max_center = predict(frame, target_class='cell phone')  
                PD_frame, max_center = predict(frame, target_class='B1')  # 这里可以更改目标类别

                logger.info(f'坐标：{max_center}')

                cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
                cv2.imshow("frame", PD_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cv2.destroyAllWindows()
    if cap is not None:
        cap.release()