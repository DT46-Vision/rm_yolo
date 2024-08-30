import cv2
import numpy as np
import time
from ultralytics import YOLO

# 原生YOLO模型路径
native_model_path = "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best.pt"
# OpenVINO模型路径
ov_model_path = (
    "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/models/best_openvino_model/"
)
# 输入图像路径
image_path = "/home/morefine/ros_ws/src/rm_yolo_aim/rm_yolo_aim/image.jpg"

# 加载原生YOLO模型
native_model = YOLO(native_model_path)

# 加载OpenVINO模型
ov_model = YOLO(ov_model_path)

# 读取图像
image = cv2.imread(image_path)

# 计时原生YOLO推理
start_time_native = time.time()
results_native = native_model(image)
end_time_native = time.time()
native_inference_time = end_time_native - start_time_native

# 计时OpenVINO推理
start_time_ov = time.time()
results_ov = ov_model(image)
end_time_ov = time.time()
ov_inference_time = end_time_ov - start_time_ov

# 输出结果
print(f"原生YOLO推理时间: {native_inference_time:.4f}秒")
print(f"OpenVINO推理时间: {ov_inference_time:.4f}秒")

{
    "3": {
        "confidence": 0.8910550475120544,
        "bbox": [
            [272.8251953125, 271.052001953125, 441.20977783203125, 402.747802734375]
        ],
    },
    "3": {
        "confidence": 0.8910550475120544,
        "bbox": [
            [272.8251953125, 271.052001953125, 441.20977783203125, 402.747802734375]
        ],
    },
}
