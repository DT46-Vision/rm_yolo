import math
from loguru import logger

# 定义常量, 弧度转角度
RAD2DEG = 180 / math.pi
DEG2RAD = math.pi / 180

def pick_armor(armors_dict, color):

    filtered_height_data = {k: v for k, v in armors_dict.items() if v["height"] > 20}

    if color == 0:
        filtered_color_data = {k: v for k, v in filtered_height_data.items() if v["class_id"] < 7}

    elif color == 1:
        filtered_color_data = {k: v for k, v in filtered_height_data.items() if v["class_id"] > 6}

    if not filtered_color_data:
        tracking_armor = {}
    else:
        # 找到 center 中 x 绝对值最小的条目
        # min_abs_center_key = min(filtered_color_data.items(), key=lambda k: abs(filtered_color_data[k]["center"][0]))
        # min_abs_center_value = filtered_color_data[min_abs_center_key]

        # 在剩余选项中找出 height 最大的条目
        tracking_armor = max(filtered_color_data.items(), key=lambda item: item[1]["height"])[1]

    logger.info(f"tracking_armor: {tracking_armor}")

    return tracking_armor

def pixel_to_angle(tracking_armor, vfov):
    # 哨兵相机 水平 FOV 72°，垂直 FOV 60°
    # 步兵相机 待测量
    height = tracking_armor["height"]
    center = tracking_armor["center"]

    # 估计距离
    deep   = height

    # 相机 x, y 坐标系下投影面的 Z 轴距离
    focal_pixel_distance = ((center[0] / math.tan(vfov / 2 * DEG2RAD)))

    # 计算角度
    yaw    = math.atan(center[0] / focal_pixel_distance) * RAD2DEG
    pitch  = math.atan(center[1] / focal_pixel_distance) * RAD2DEG

    return yaw, pitch, deep



if __name__ == "__main__":

    armors_dict = {
        "179": {"class_id": 7, "height": 290, "center": [179, 35]},
        "-143": {"class_id": 3, "height": 288, "center": [-143, -35]},
        "149": {"class_id": 3, "height": 191, "center": [149, 36]},
        "-113": {"class_id": 2, "height": 300, "center": [640, -35]},
    }

    result = pick_armor(armors_dict,0)

    tracking_armor_data = pixel_to_angle(result, 72)
    
    logger.info(f"tracking_armor_data: {tracking_armor_data}")