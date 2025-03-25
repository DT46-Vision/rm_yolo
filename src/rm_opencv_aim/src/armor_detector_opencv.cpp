// armor_detector_opencv.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <thread>
#include <chrono>
#include <jsoncpp/json/json.h> // 需要引入 JSON 库
#include <utility> // 包含 std::pair
#include <vector>
#include <limits> // 用于 std::numeric_limits
#include <algorithm> // 用于 std::min_element 和 std::max_element
#include <set>

// 定义计算距离的函数
double calculate_distance(const std::pair<double, double>& point1, const std::pair<double, double>& point2) {
    double x1 = point1.first;  // 解包第一个点的 x 坐标
    double y1 = point1.second; // 解包第一个点的 y 坐标
    double x2 = point2.first;  // 解包第二个点的 x 坐标
    double y2 = point2.second;  // 解包第二个点的 y 坐标

    // 使用距离公式计算并返回结果
    double distance = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    return distance;
}

// 调整矩形的函数
std::pair<std::pair<double, double>, double> adjust(const std::pair<double, double>& w_h, double angle) {
    double w = w_h.first;  // 解包宽度
    double h = w_h.second; // 解包高度

    if (w > h) {  // 如果宽度大于高度
        std::swap(w, h);  // 交换宽度和高度

        // 调整角度，使其跟随高度
        if (angle >= 0) {
            angle -= 90;  // 减去 90 度
        } else {
            angle += 90;  // 加上 90 度
        }
    }

    return {{w, h}, angle}; // 返回调整后的结果
}

// 将角度转换为斜率的函数
double angle_to_slope(double angle_degrees) {
    double angle_radians = angle_degrees * M_PI / 180.0; // 将角度转换为弧度
    double slope = std::tan(angle_radians); // 计算斜率
    return slope; // 返回斜率
}

// 投影计算函数
std::pair<double, double> project(const std::vector<std::pair<double, double>>& polygon, const std::pair<double, double>& axis) {
    // 存储投影结果
    std::vector<double> projections;

    // 计算每个点在投影轴上的投影
    for (const auto& point : polygon) {
        double projection = (point.first * axis.first + point.second * axis.second); // 计算点在轴上的投影
        projections.push_back(projection); // 将投影结果存储到 vector 中
    }

    // 计算最小值和最大值
    double min_projection = *std::min_element(projections.begin(), projections.end());
    double max_projection = *std::max_element(projections.begin(), projections.end());

    return {min_projection, max_projection}; // 返回最小值和最大值
}

// 检查两个多边形是否重叠的函数
bool is_coincide(const std::vector<std::pair<double, double>>& a, const std::vector<std::pair<double, double>>& b) {
    // 遍历多边形 a 和 b
    for (const auto& polygon : {a, b}) { 
        for (size_t i = 0; i < polygon.size(); ++i) { // 遍历每个多边形的边
            auto p1 = polygon[i]; // 获取当前点
            auto p2 = polygon[(i + 1) % polygon.size()]; // 获取下一个点（循环返回到第一个点）

            // 计算法向量
            std::pair<double, double> normal = {p2.second - p1.second, p1.first - p2.first};

            // 计算多边形 a 和 b 的投影的最小值和最大值
            auto [min_a, max_a] = project(a, normal);
            auto [min_b, max_b] = project(b, normal);

            // 检查是否相交
            if (max_a < min_b || max_b < min_a) { // 不相交
                return false; // 返回 false
            }
        }
    }
    return true; // 如果没有早期返回，说明两个多边形相交
}

class Light {
public:
    int cx;          // 中心 x 坐标
    int cy;          // 中心 y 坐标
    double height;   // 灯条高度
    std::pair<double, double> up;    // 上端点
    std::pair<double, double> down;  // 下端点
    double angle;    // 灯条角度
    int color;       // 灯条颜色

    // 构造函数
    Light(const std::pair<double, double>& up, const std::pair<double, double>& down, double angle, int color) 
        : up(up), down(down), angle(angle), color(color) {
        cx = static_cast<int>(std::abs(up.first - down.first) / 2 + std::min(up.first, down.first)); // 计算中心 x 坐标
        cy = static_cast<int>(std::abs(up.second - down.second) / 2 + std::min(up.second, down.second)); // 计算中心 y 坐标
        height = calculate_distance(up, down); // 计算高度
    }

private:
    // 计算两个点之间的距离
    double calculate_distance(const std::pair<double, double>& point1, const std::pair<double, double>& point2) {
        return std::sqrt(std::pow(point2.first - point1.first, 2) + std::pow(point2.second - point1.second, 2)); // 使用距离公式
    }
};

class Armor {
public:
    std::pair<int, int> center; // 中心坐标
    std::pair<double, double> light1_up;   // 第一根灯条的上端点
    std::pair<double, double> light1_down; // 第一根灯条的下端点
    std::pair<double, double> light2_up;   // 第二根灯条的上端点
    std::pair<double, double> light2_down; // 第二根灯条的下端点
    int color;        // 装甲板颜色
    double height;    // 装甲板高度
    int type;         // 装甲板类型

    // 构造函数
    Armor(const Light& light1, const Light& light2, double height, int type)
        : light1_up(light1.up), light1_down(light1.down),
          light2_up(light2.up), light2_down(light2.down),
          height(height), type(type) {
        
        int armor_cx = static_cast<int>(std::abs(light1.cx - light2.cx) / 2 + std::min(light1.cx, light2.cx)); // 计算中心 x 坐标
        int armor_cy = static_cast<int>(std::abs(light1.cy - light2.cy) / 2 + std::min(light1.cy, light2.cy)); // 计算中心 y 坐标
        center = {armor_cx, armor_cy}; // 设置中心坐标
        color = light1.color; // 装甲板颜色初始化为 light1 的颜色
    }

    // 类型分类函数
    int type_class() const {
        if (color == 0) {
            if (type == 0) {
                return 7;
            } else if (type == 1) {
                return 6;
            }
        } else if (color == 1) {
            if (type == 0) {
                return 1;
            } else if (type == 1) {
                return 0;
            }
        }
        return -1; // 默认返回值
    }
};

class ArmorDetector {
public:
    cv::Mat img; // 原始图像
    cv::Mat img_binary; // 二值化图像
    cv::Mat img_draw; // 绘制图像
    std::vector<Light> lights; // 存储灯条列表
    std::vector<Armor> armors; // 存储装甲板列表
    std::map<int, std::map<std::string, int>> armors_dict; // 装甲板信息字典
    int binary_val; // 二值化阈值
    int color; // 颜色模式
    int display_mode; // 显示模式
    std::map<std::string, int> light_params; // 灯条参数
    std::map<std::string, int> armor_color; // 装甲板颜色映射
    std::map<std::string, int> light_color; // 灯条颜色映射
    std::map<std::string, int> light_dot; // 灯条中心点颜色映射
    // 构造函数
    ArmorDetector(int detect_color, int display_mode, int binary_val, 
                  const std::map<std::string, int>& light_params, 
                  const std::map<std::string, int>& color_params): 
        binary_val(binary_val), color(detect_color), display_mode(display_mode), 
        light_params(light_params), armor_color(color_params.at("armor_color")),
        light_color(color_params.at("light_color")), light_dot(color_params.at("light_dot")) {}
};

int main() {
    // // 前向声明 project 函数
    // std::pair<double, double> project(const std::vector<std::pair<double, double>>& polygon, const std::pair<double, double>& axis);

    // 假设 Light 类已经定义并创建了 light1 和 light2 实例
    Light light1({1.0, 4.0}, {1.0, 1.0}, 30.0, 0); // 创建第一根灯条
    Light light2({3.0, 4.0}, {3.0, 1.0}, 30.0, 0); // 创建第二根灯条
    
    Armor armor(light1, light2, 5.0, 1); // 创建装甲板

    std::cout << "Armor center: (" << armor.center.first << ", " << armor.center.second << ")" << std::endl;
    std::cout << "Armor type class: " << armor.type_class() << std::endl;

    return 0;
}