import math
from loguru import logger

# 定义常量, 弧度转角度
RAD2DEG = 180 / math.pi
DEG2RAD = math.pi / 180

def select_tracking_armor(armors_dict, color):

    # 筛选垂直方向的长度大于20像素的装甲板
    filtered_height_data = {k: v for k, v in armors_dict.items() if v["height"] > 1}

    # 根据颜色筛选 1: 蓝色, 0: 红色
    if color == 1:
        filtered_color_data = {k: v for k, v in filtered_height_data.items() if v["class_id"] < 6}
    elif color == 0:
        filtered_color_data = {k: v for k, v in filtered_height_data.items() if v["class_id"] > 5}
        
    # 最终筛选
    if not filtered_color_data:
        tracking_armor = {}
    else:
        # 找出 height 最大的条目
        tracking_armor = max(filtered_color_data.items(), key=lambda item: item[1]["height"])[1]

    return tracking_armor

def pixel_to_angle_and_deep(height, center, vfov, pic_width):

    # 估计距离
    deep = height
    # 确保 vfov 是以弧度为单位
    vfov_radians = vfov * DEG2RAD
    # 相机 x, y 坐标系下投影面的 Z 轴距离(单位: 像素)
    focal_pixel_distance = (pic_width / 2) / math.tan(vfov_radians / 2)
    # 确保 focal_pixel_distance 不为零
    if focal_pixel_distance == 0:
        focal_pixel_distance = 0.000_000_1
    # 计算角度
    yaw   = math.atan(center[0] / focal_pixel_distance) * RAD2DEG
    pitch = math.atan(center[1] / focal_pixel_distance) * RAD2DEG
    return yaw, pitch, deep

if __name__ == "__main__":

    armors_dict = {
        "179":  {"class_id": 7, "height": 290, "center": [ 1,  333]},
        "-143": {"class_id": 3, "height": 288, "center": [-143, -35]},
        "149":  {"class_id": 3, "height": 191, "center": [ 149,  36]},
        "-113": {"class_id": 2, "height": 300, "center": [ 91, -35]},
    }

    result = select_tracking_armor(armors_dict, 0)

    yaw, pitch, deep = pixel_to_angle_and_deep(result["height"], result["center"], 72, 1080)
    
    logger.info(f"yaw: {yaw:.2f}, pitch: {pitch:.2f}, deep: {deep:.2f}")