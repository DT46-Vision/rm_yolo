import numpy as np
import cv2

class KalmanFilter:
    def __init__(self, dt):
        # 初始化卡尔曼滤波器
        self.kf = cv2.KalmanFilter(4, 2)  # 状态维度为4，观测维度为2
        
        # 状态转移矩阵
        self.kf.transitionMatrix = np.array([[1, 0, dt, 0],
                                              [0, 1, 0, dt],
                                              [0, 0, 1, 0],
                                              [0, 0, 0, 1]], np.float32)
        
        # 观测矩阵
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                               [0, 1, 0, 0]], np.float32)
        
        # 过程噪声协方差矩阵
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.1
        
        # 观测噪声协方差矩阵
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        
        # 初始状态
        self.kf.statePost = np.zeros((4, 1), np.float32)
        self.deep = 0

    def predict(self):
        # 进行预测
        return self.kf.predict()

    def update(self, yaw, pitch):
        # 更新状态
        measurement = np.array([[yaw], [pitch]], np.float32)
        self.kf.correct(measurement)

    def get_state(self):
        # 获取当前状态（yaw, pitch）
        return self.kf.statePost.flatten()[:2]  # 仅返回yaw和pitch

# 示例用法
if __name__ == "__main__":
    dt = 0.1  # 时间步长
    kf = KalmanFilter(dt)

    # 模拟传入的yaw和pitch数据
    measurements = [(1.0, 0.5),
                    (1.2, 0.6),
                    (1.1, 0.55)]

    for yaw, pitch in measurements:
        kf.predict()  # 进行预测
        kf.update(yaw, pitch)  # 更新状态
        predicted_state = kf.get_state()  # 获取预测的状态
    print(f"预测的 yaw: {predicted_state[0]:.2f}, pitch: {predicted_state[1]:.2f}")