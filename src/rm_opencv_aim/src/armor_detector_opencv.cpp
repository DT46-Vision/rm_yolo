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
#include <algorithm> // 用于 std::min_element 和 std::max_element

// 计算两个点之间的距离
double calculate_distance(const cv::Point2f& p1, const cv::Point2f& p2) {
    return cv::norm(p1 - p2); // 使用 OpenCV 计算距离
}

// 调整宽高和角度的函数
std::pair<cv::Size2f, double> adjust(const cv::Size2f& w_h, double angle) {
    // 解包宽度和高度
    float w = w_h.width;  // 使用 cv::Size2f 的 width 成员
    float h = w_h.height; // 使用 cv::Size2f 的 height 成员

    if (w > h) {  // 如果宽度大于高度
        std::swap(w, h);  // 交换宽度和高度

        // 调整角度，使其跟随高度
        if (angle >= 0) {
            angle -= 90;  // 减去 90 度
        } else {
            angle += 90;  // 加上 90 度
        }
    }

    return std::make_pair(cv::Size2f(w, h), angle); // 返回调整后的结果
}

// 将角度转换为斜率的函数
double angle_to_slope(double angle_degrees) {
    double angle_radians = angle_degrees * M_PI / 180.0; // 将角度转换为弧度
    double slope = std::tan(angle_radians); // 计算斜率
    return slope; // 返回斜率
}

// 投影计算函数
std::pair<float, float> project(const std::vector<cv::Point2f>& polygon, const cv::Point2f& axis) {
    float min = std::numeric_limits<float>::max(); // 初始化最小值
    float max = std::numeric_limits<float>::lowest(); // 初始化最大值

    for (const auto& point : polygon) {
        // 计算点在法向量上的投影
        float projection = (point.x * axis.x + point.y * axis.y); // 点在法向量上的投影
        min = std::min(min, projection); // 更新最小值
        max = std::max(max, projection); // 更新最大值
    }
    return {min, max}; // 返回最小值和最大值
}

//检查两个多边形是否重叠
bool is_coincide(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b) {
    // 检查多边形 a 和 b 是否有效
    if (a.size() < 3 || b.size() < 3) {
        return false; // 如果其中一个多边形无效，直接返回 false
    }

    // 遍历多边形 a 和 b
    for (const auto& polygon : {a, b}) {
        for (size_t i = 0; i < polygon.size(); ++i) {
            auto p1 = polygon[i]; // 当前点
            auto p2 = polygon[(i + 1) % polygon.size()]; // 下一个点

            // 计算法向量
            cv::Point2f normal = {p2.y - p1.y, p1.x - p2.x}; // 计算法向量

            // 计算投影的最小值和最大值
            auto [min_a, max_a] = project(a, normal);
            auto [min_b, max_b] = project(b, normal);

            // 检查是否相交
            if (max_a < min_b || max_b < min_a) {
                return false; // 不相交
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
    cv::Point2f up;  // 上端点
    cv::Point2f down;// 下端点
    double angle;    // 灯条角度
    int color;       // 灯条颜色

    // 构造函数
    Light(const cv::Point2f& up, const cv::Point2f& down, double angle, int color)
        : up(up), down(down), angle(angle), color(color) {
        cx = static_cast<int>((up.x + down.x) / 2); // 计算中心 x 坐标
        cy = static_cast<int>((up.y + down.y) / 2); // 计算中心 y 坐标
        height = calculate_distance(up, down); // 计算高度
    }
};

class Armor {
public:
    cv::Point center;      // 中心坐标
    cv::Point2f light1_up;   // 第一根灯条的上端点
    cv::Point2f light1_down; // 第一根灯条的下端点
    cv::Point2f light2_up;   // 第二根灯条的上端点
    cv::Point2f light2_down; // 第二根灯条的下端点
    int color;              // 装甲板颜色
    double height;          // 装甲板高度
    int type;               // 装甲板类型

    // 构造函数
    Armor(const Light& light1, const Light& light2, double height, int type)
        : light1_up(light1.up), light1_down(light1.down),
          light2_up(light2.up), light2_down(light2.down),
          height(height), type(type) {
        
        // 计算中心坐标
        int armor_cx = static_cast<int>((light1.cx + light2.cx) / 2);
        int armor_cy = static_cast<int>((light1.cy + light2.cy) / 2);
        center = cv::Point(armor_cx, armor_cy); // 设置中心坐标
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

    // 颜色映射类型定义
    using ColorMap = std::map<int, std::array<int, 3>>; // 使用 std::array 表示 RGB 颜色
    using ColorParams = std::map<std::string, ColorMap>; // 使用嵌套的 map 类型

    // 成员变量
    ColorMap armor_color; // 装甲板颜色映射
    ColorMap light_color; // 灯条颜色映射
    ColorMap light_dot;   // 灯条中心点颜色映射
    
    // 构造函数
    ArmorDetector(int detect_color, int display_mode, int binary_val, 
                  const std::map<std::string, int>& light_params, 
                  const ColorParams& color_params) 
        : binary_val(binary_val), color(detect_color), display_mode(display_mode), 
          light_params(light_params),
          armor_color(color_params.at("armor_color")), // 获取装甲板颜色映射
          light_color(color_params.at("light_color")), // 获取灯条颜色映射
          light_dot(color_params.at("light_dot")) {}   // 获取灯条中心点颜色映射
    
    // 处理图像的函数
    cv::Mat process(const cv::Mat& img_input) {
        img = img_input.clone(); // 复制输入图像
        cv::Mat gray_img; // 用于存储灰度图像
        // 将图像转换为灰度图并进行二值化处理
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY); // 转为灰度图
        cv::threshold(gray_img, img_binary, binary_val, 255, cv::THRESH_BINARY); // 二值化处理
        return img_binary; // 返回二值化图像
    }

    // 查找灯条的函数
    std::vector<Light> find_lights(const cv::Mat& img_binary_input) {
        std::vector<std::vector<cv::Point>> contours; // 存储轮廓
        std::vector<cv::RotatedRect> is_lights; // 存储处理后的灯条坐标
        std::vector<cv::RotatedRect> is_lights_filtered; // 存储过滤后的灯条坐标
        std::vector<Light> lights_found; // 存储过滤后的灯条坐标

        // 查找轮廓，不使用层级信息
        cv::findContours(img_binary_input, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 遍历轮廓，查找灯条
        for (const auto& contour : contours) {
            // 计算轮廓面积
            if (cv::contourArea(contour) >= light_params["light_area_min"]) {
                // 获取最小外接矩形
                cv::RotatedRect min_rect = cv::minAreaRect(contour);
                cv::Size2f w_h = min_rect.size; // 矩形的宽高
                double angle = min_rect.angle; // 矩形的旋转角度

                // 调整宽高和角度
                std::make_pair(w_h, angle) = adjust(w_h, angle); // 假设有 adjust 函数

                // 检查角度是否在指定范围内
                if (angle >= light_params["light_angle_min"] && angle <= light_params["light_angle_max"]) {
                        // 添加合适的矩形到is_lights
                        cv::RotatedRect rect(min_rect.center, w_h, static_cast<float>(angle)); // 创建旋转矩形
                        is_lights.push_back(rect); // 存储旋转矩形
                }
            }
        }

        // // 过滤不重叠的光源
        // for (const auto& is_light : is_lights) { // 遍历所有光源
        //     bool is_overlapping = false; // 标记当前光源是否与其他光源重叠

        //     // 获取当前光源的边界框点
        //     std::vector<cv::Point2f> current_box(4); 
        //     cv::boxPoints(is_light, current_box); 

        //     // 遍历所有其他光源
        //     for (const auto& other_is_light : is_lights) { 
        //         // 确保不是同一个光源
        //         if (is_light.center != other_is_light.center) {
        //             // 获取其他光源的边界框点
        //             std::vector<cv::Point2f> other_box(4); 
        //             cv::boxPoints(other_is_light, other_box); 

        //             // 检查边界框是否有效
        //             if (current_box.size() == 4 && other_box.size() == 4) {
        //                 // 检查是否重叠
        //                 if (is_coincide(current_box, other_box)) {
        //                     is_overlapping = true; // 设置为重叠标记
        //                     break; // 找到重叠后跳出内层循环
        //                 }
        //             } else {
        //                 std::cerr << "Warning: Invalid box size detected!" << std::endl; // 调试信息
        //             }
        //         }
        //     }

        //     // 如果没有重叠，添加到不重叠的灯条列表中
        //     if (!is_overlapping) { 
        //         is_lights_filtered.push_back(is_light); 
        //     }
        // }
        // return lights;
        // std::cout << "Detected lights (is_lights):" << std::endl;
        // for (const auto& light : is_lights) {
        //     std::cout << "Center: (" << light.center.x << ", " << light.center.y << "), "
        //     << "Size: (" << light.size.width << ", " << light.size.height << "), "
        //     << "Angle: " << light.angle << std::endl;
        // }

        for (const auto& rect : is_lights) {  // 遍历过滤后的灯条
            cv::Point2f box[4];
            rect.points(box); // 获取旋转矩形的四个点

            // 计算上端点
            int right_up_x = static_cast<int>(box[0].x);
            int right_up_y = static_cast<int>(box[0].y);
            int left_up_x = static_cast<int>(box[3].x);
            int left_up_y = static_cast<int>(box[3].y);

            int up_x = static_cast<int>(std::abs(right_up_x - left_up_x) / 2 + std::min(right_up_x, left_up_x));
            int up_y = static_cast<int>(std::abs(right_up_y - left_up_y) / 2 + std::min(right_up_y, left_up_y));
            cv::Point2f up(up_x, up_y);

            // 计算下端点
            int right_down_x = static_cast<int>(box[1].x);
            int right_down_y = static_cast<int>(box[1].y);
            int left_down_x = static_cast<int>(box[2].x);
            int left_down_y = static_cast<int>(box[2].y);

            int down_x = static_cast<int>(std::abs(right_down_x - left_down_x) / 2 + std::min(right_down_x, left_down_x));
            int down_y = static_cast<int>(std::abs(right_down_y - left_down_y) / 2 + std::min(right_down_y, left_down_y));
            cv::Point2f down(down_x, down_y);

            int length = static_cast<int>(std::sqrt(std::pow(down_x - up_x, 2) + std::pow(down_y - up_y, 2))); // 计算线段的长度
            cv::Mat roi(1, length, CV_8UC3, cv::Scalar(0, 0, 0)); // 创建新图像以存储裁剪的线段像素

            // 计算线段上的每个像素
            for (int i = 0; i < length; ++i) {
                float t = static_cast<float>(i) / length; // 计算比例
                int current_x = static_cast<int>(up_x + (down_x - up_x) * t);
                int current_y = static_cast<int>(up_y + (down_y - up_y) * t);

                // 添加边界检查
                if (0 <= current_x && current_x < img.cols && 0 <= current_y && current_y < img.rows) {
                    roi.at<cv::Vec3b>(0, i) = img.at<cv::Vec3b>(current_y, current_x); // 保存像素值
                }
            }

            // 计算红色和蓝色的总和
            int sum_b = cv::sum(roi)[0]; // 蓝色通道总和
            int sum_r = cv::sum(roi)[2]; // 红色通道总和

            // 根据模式识别颜色
            if ((color == 1 || color == 2) && sum_b > sum_r * light_params["light_blue_ratio"]) {
                Light light_blue(up, down, rect.angle, 1); // 创建蓝色灯条对象
                lights_found.push_back(light_blue); // 添加蓝色灯条
            } else if ((color == 0 || color == 2) && sum_r > sum_b * light_params["light_red_ratio"]) {
                Light light_red(up, down, rect.angle, 0); // 创建红色灯条对象
                lights_found.push_back(light_red); // 添加红色灯条
            }
        }
        lights = lights_found;
        return lights;
    }


    
};

int main() {
    // // 前向声明 project 函数
    // std::pair<double, double> project(const std::vector<std::pair<double, double>>& polygon, const std::pair<double, double>& axis);

// 创建 ArmorDetector 对象
    std::map<std::string, int> light_params = {
        {"light_area_min", 5},
        {"light_angle_min", -35},
        {"light_angle_max", 35},
        {"light_red_ratio", 1},
        {"light_blue_ratio", 1},
        {"cy_tol", 5},
        {"height_tol", 18}, 
        {"light_angle_tol", 7},
        {"vertical_discretization", 2.1},
        {"height_multiplier", 2.7},
    };

    std::map<std::string, std::map<int, std::array<int, 3>>> color_params = {
        {"armor_color", {{1, {255, 255, 0}}, {0, {128, 0, 128}}}},
        {"light_color", {{1, {200, 71, 90}}, {0, {0, 100, 255}}}},
        {"light_dot", {{1, {0, 0, 255}}, {0, {255, 0, 0}}}}
    };
    //模式参数字典
    int detect_color =  1;  // 颜色参数 0: 识别红色装甲板, 1: 识别蓝色装甲板, 2: 识别全部装甲板
    int display_mode = 1; // 显示模式 0: 不显示, 1: 显示二值化图, 2: 显示二值化图和结果图像
    // 图像参数字典
    int binary_val = 225;
    ArmorDetector detector(detect_color, display_mode, binary_val, light_params, color_params); // 创建 ArmorDetector 对象

    // 读取输入图像
    cv::Mat input_image = cv::imread("./src/rm_opencv_aim/test/b.jpg");
    if (input_image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1; // 返回错误码
    }

    // 处理图像并获取二值化结果
    cv::Mat binary_image = detector.process(input_image);
    std::vector<Light> lights; // 存储轮廓
    lights = detector.find_lights(binary_image);
    // 处理找到的灯条（例如，输出数量）
    std::cout << "找到的灯条数量: " << lights.size() << std::endl;
    //创建窗口并显示图像
    cv::namedWindow("Input Image", cv::WINDOW_AUTOSIZE); // 创建窗口
    cv::imshow("Input Image", input_image); // 显示输入图像

    cv::namedWindow("Binary Image", cv::WINDOW_AUTOSIZE); // 创建窗口
    cv::imshow("Binary Image", binary_image); // 显示二值化图像

    // 等待用户按键
    cv::waitKey(0); // 等待任意按键

    // 关闭所有窗口
    cv::destroyAllWindows();
    return 0;
}