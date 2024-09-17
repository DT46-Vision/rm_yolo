import cv2
import numpy as np
from ultralytics import YOLO

# 加载YOLO模型
model = YOLO(
    "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt"
)  # 预训练的YOLOv8n模型

# 定义一个函数来计算中点坐标
def calculate_center(box):
    x_center = int((box[0] + box[2]) / 2)
    y_center = int((box[1] + box[3]) / 2)
    return (x_center, y_center)

# 定义检测函数
def detect_and_get_center(input_str):
    # 读取图像
    img = cv2.imread("/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/image.jpg")

    # 进行预测
    results = model.predict(img)
    
    max_perimeter = 0
    max_center = None
    
    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)  # 获取类别 ID
            label = model.names[class_id]  # 获取类别名称
            
            # 将 box.xyxy 转换为列表
            box_coords = box.xyxy.tolist()[0]  # 取第一个边界框的坐标
            
            # 计算周长
            perimeter = 2 * (box_coords[2] - box_coords[0]) + 2 * (box_coords[3] - box_coords[1])
            
            # 根据输入进行判断
            if input_str in label:
                center = calculate_center(box_coords)
                if perimeter > max_perimeter:
                    max_perimeter = perimeter
                    max_center = center
    
    return max_center if max_center else "未检测到目标"

# 示例用法
print(detect_and_get_center("B2"))  
print(detect_and_get_center("B"))   