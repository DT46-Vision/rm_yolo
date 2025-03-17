import math
from loguru import logger

# 定义常量, 弧度转角度
RAD2DEG = 180 / math.pi
DEG2RAD = math.pi / 180

def select_tracking_armor(armors_dict, color, track_hight_tol, cx_tol):

    # 筛选垂直方向的长度大于1像素的装甲板
    filtered_height_data = {k: v for k, v in armors_dict.items() if v["height"] > 1}
    
    # 没有直接退出
    if not filtered_height_data:
        tracking_armor = {}
        return tracking_armor
    
    # 根据颜色筛选 1: 蓝色, 0: 红色
    if color == 1:
        filtered_color_data = {k: v for k, v in filtered_height_data.items() if v["class_id"] < 6}
    
    elif color == 0:
        filtered_color_data = {k: v for k, v in filtered_height_data.items() if v["class_id"] > 5}
    
    else:
        return {}  # 颜色输入不合法就返回空
    
    # 没有直接退出
    if not filtered_color_data:
        tracking_armor = {}
        return tracking_armor
    
    # 检查 filtered_color_data 的数量
    if len(filtered_color_data) == 1:
        tracking_armor = next(iter(filtered_color_data.values()))
        return tracking_armor

    # 按高度排序并取前两个条目
    top_two_highest_armor = sorted(filtered_color_data.items(), key=lambda item: item[1]["height"], reverse=True)[:2]
    # 检查高度差
    height_diff = top_two_highest_armor[0][1]["height"] - top_two_highest_armor[1][1]["height"]
    if height_diff > track_hight_tol:
        # 如果高度差超过阈值，则直接返回最高的装甲板
        return top_two_highest_armor[0][1]
    else:
        # 检查 X 坐标差
        cx_diff = abs(top_two_highest_armor[0][1]['center'][0] - top_two_highest_armor[1][1]['center'][0])
        if cx_diff > cx_tol:
            # 如果 X 坐标差超过阈值，则返回 X 坐标最小的装甲板
            if abs(top_two_highest_armor[0][1]['center'][0]) < abs(top_two_highest_armor[1][1]['center'][0]):
                return top_two_highest_armor[0][1]
            else:
                return top_two_highest_armor[1][1]
        else:
            # 否则，选择 Y 坐标最高的中心点对应的装甲板
            if top_two_highest_armor[0][1]['center'][1] > top_two_highest_armor[1][1]['center'][1]:
                return top_two_highest_armor[0][1]
            else:
                return top_two_highest_armor[1][1]

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
        "179":  {"class_id": 7, "height": 300, "center": [ 147,  333]},
        "-143": {"class_id": 1, "height": 288, "center": [-143, -35]},
        "175":  {"class_id": 7, "height": 296, "center": [ 149,  36]},
        "-113": {"class_id": 1, "height": 300, "center": [ 91, -35]},
    }

    result = select_tracking_armor(armors_dict, 0)
    if result:
        yaw, pitch, deep = pixel_to_angle_and_deep(result["height"], result["center"], 55, 72)
    
        logger.info(f"yaw: {yaw:.2f}, pitch: {pitch:.2f}, deep: {deep:.2f}")