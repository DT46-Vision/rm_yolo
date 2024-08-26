import json
import cv2
import numpy as np
from ultralytics import YOLO

from loguru import logger

# 加载YOLO模型
model = YOLO(
    "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt"
)  # 预训练的YOLOv8n模型

# 运行推理
results = model("/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/image.jpg")  # 输入图像

# 初始化一个列表来存储检测结果
detections = []

# 处理结果
for result in results:
    boxes = result.boxes  # 获取边界框

    # 遍历每个检测到的边界框
    for i, box in enumerate(boxes.xyxy):  # xyxy格式的边界框
        detection = {
            "class": int(boxes.cls[i]),  # 类别
            "confidence": float(boxes.conf[i]),  # 置信度
            "box": box.tolist(),  # 边界框坐标
        }
        detections.append(detection)

        # 在图像上绘制边界框
        x1, y1, x2, y2 = map(int, box)  # 将坐标转换为整数
        cv2.rectangle(result.orig_img, (x1, y1), (x2, y2), (0, 255, 255), 1)  # 绘制边界框
        cv2.putText(
            result.orig_img,
            f'Class: {detection["class"]}, Conf: {detection["confidence"]:.2f}',
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (0, 255, 255),
            1,
        )


# 将检测结果转换为JSON格式
json_output = json.dumps(detections, indent=4)

# 保存JSON结果到文件
with open("detections.json", "w") as json_file:
    json_file.write(json_output)

# 保存结果图像
cv2.imwrite("result.jpg", result.orig_img)

