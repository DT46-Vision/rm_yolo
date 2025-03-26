// armor_detector_opencv.cpp
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <thread>
#include <chrono>
#include <utility> // 包含 std::pair
#include <limits> // 用于 std::numeric_limits
#include <algorithm> // 用于 std::min_element 和 std::max_element
#include <set>

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
        cx = static_cast<int>(std::abs(up.x - down.x) / 2 + std::min(up.x, down.x)); // 计算中心 x 坐标
        cy = static_cast<int>(std::abs(up.y - down.y) / 2 + std::min(up.y, down.y)); // 计算中心 y 坐标
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
        //计算中心坐标
        int armor_cx = static_cast<int>(std::abs(light1.cx - light2.cx) / 2 + std::min(light1.cx, light2.cx));
        int armor_cy = static_cast<int>(std::abs(light1.cy - light2.cy) / 2 + std::min(light1.cy, light2.cy));
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

// 定义 Armor_info 结构体
typedef struct {
    float height; // 装甲板高度
    int class_id; // 装甲板类别 ID
    float cx; // 中心 x 坐标
    float cy; // 中心 y 坐标
} Armor_info;

// 定义 Light_params 结构体
typedef struct {
    int light_area_min;
    int light_angle_min;
    int light_angle_max;
    float light_red_ratio;
    float light_blue_ratio;
    int cy_tol;
    int height_tol; 
    int light_angle_tol;
    float vertical_discretization;
    float height_multiplier;
} Light_params;

class ArmorDetector {
public:
    cv::Mat img; // 原始图像
    cv::Mat img_binary; // 二值化图像
    cv::Mat img_drawn; // 绘制图像
    std::vector<Light> lights; // 存储灯条列表
    std::vector<Armor> armors; // 存储装甲板列表
    std::vector<Armor_info> armors_info; // 装甲板信息字典

    int binary_val; // 二值化阈值
    int color; // 颜色模式
    int display_mode; // 显示模式
    Light_params light_params; // 灯条参数
    
    // 构造函数
    ArmorDetector(int detect_color, int display_mode, int binary_val, 
                const Light_params light_params) 
                : color(detect_color), binary_val(binary_val), display_mode(display_mode), light_params(light_params){}
    
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
        //success!!!
        //遍历轮廓，查找灯条
        for (const auto& contour : contours) {
            // 计算轮廓面积
            if (cv::contourArea(contour) >= light_params.light_area_min) {
                // 获取最小外接矩形
                cv::RotatedRect min_rect = cv::minAreaRect(contour);
            
                cv::Size2f w_h = min_rect.size; // 矩形的宽高
                double angle = min_rect.angle; // 矩形的旋转角度

                // 调整宽高和角度
                std::tie(w_h, angle) = adjust(w_h, angle); // 假设有 adjust 函数
                //success!

                // 检查角度是否在指定范围内
                if (angle >= light_params.light_angle_min && angle <= light_params.light_angle_max) {
                        // 添加合适的矩形到is_lights
                        cv::RotatedRect rect(min_rect.center, w_h, static_cast<float>(angle)); // 创建旋转矩形
                        is_lights.push_back(rect); // 存储旋转矩形
                        //std::cout << "angle:" << rect.angle << "w_h:" << rect.size << "angle:" << rect.center << std::endl;
               }
            }
        }
        //success!!
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
            if (current_x >= 0 && current_x < img.cols && current_y >= 0 && current_y < img.rows) {
                roi.at<cv::Vec3b>(0, i) = img.at<cv::Vec3b>(current_y, current_x); // 保存像素值
            }

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

                if (0 <= current_x && current_x < img.cols && 0 <= current_y && current_y < img.rows) {
                    roi.at<cv::Vec3b>(0, i) = img.at<cv::Vec3b>(current_y, current_x); // 保存像素值
                }
            }

            // 计算红色和蓝色的总和
            int sum_b = cv::sum(roi)[0]; // 蓝色通道总和
            int sum_r = cv::sum(roi)[2]; // 红色通道总和

            // 根据模式识别颜色
            if ((color == 1 || color == 2) && sum_b > sum_r * light_params.light_blue_ratio) {
                Light light_blue(up, down, rect.angle, 1); // 创建蓝色灯条对象
                lights_found.push_back(light_blue); // 添加蓝色灯条
            } else if ((color == 0 || color == 2) && sum_r > sum_b * light_params.light_red_ratio) {
                Light light_red(up, down, rect.angle, 0); // 创建红色灯条对象
                lights_found.push_back(light_red); // 添加红色灯条
            }
        }
        lights = lights_found;
        return lights;
    }

    std::pair<int, float> is_close(const Light& light1, const Light& light2) {
        // 检查 y 坐标的距离
        if (std::abs(light1.cy - light2.cy) < light_params.cy_tol) {
            float height = std::max(light1.height, light2.height);
            float distance = calculate_distance({light1.cx, light1.cy}, {light2.cx, light2.cy});

            if (distance > height) {
                if (distance < height * light_params.height_multiplier) {
                    return std::make_pair(0, height); // first small armor
                } else if (distance < height * 1.86f * light_params.height_multiplier) {
                    return std::make_pair(1, height); // last large armor
                }
            }
        }

        // 检查高度差
        else if (std::abs(light1.height - light2.height) <= light_params.height_tol) {
            float angle_diff = std::abs(light1.angle - light2.angle); // 计算角度差
            if (angle_diff <= light_params.light_angle_tol) { // 判断角度差是否在容忍范围内
                float light1_angle = std::atan2(light1.up.y - light1.down.y, light1.up.x - light1.down.x) * 180.0 / M_PI; // 计算连线角度
                float light2_angle = std::atan2(light2.up.y - light2.down.y, light2.up.x - light2.down.x) * 180.0 / M_PI; // 计算连线角度
                float line_angle = std::atan2(light1.cy - light2.cy, light1.cx - light2.cx) * 180.0 / M_PI; // 计算连线角度

                float slope1 = angle_to_slope(light1_angle); // 计算斜率
                float slope2 = angle_to_slope(light2_angle);
                float slope_line = angle_to_slope(line_angle);

                // 检查斜率接近
                if (std::abs(slope1 * slope_line + 1) < light_params.vertical_discretization || 
                    std::abs(slope2 * slope_line + 1) < light_params.vertical_discretization) {
                    float height = std::max(light1.height, light2.height);
                    float distance = calculate_distance({light1.cx, light1.cy}, {light2.cx, light2.cy});
                    
                    if (distance > height) {
                        if (distance < height * light_params.height_multiplier) {
                            return std::make_pair(0, height); // first small armor
                        } else if (distance < height * 1.86f * light_params.height_multiplier) {
                            return std::make_pair(1, height); // last large armor
                        }
                    }
                }
            }
        }
        return std::make_pair(-1, -1.0f); // 不满足条件则返回 -1
    }

    std::vector<Armor> is_armor(const std::vector<Light>& lights) {
        std::vector<Armor> armors_found;
        std::set<int> processed_indices; // 用于存储已处理的矩形索引
        size_t lights_count = lights.size(); // 存储列表长度，避免重复计算

        for (size_t i = 0; i < lights_count; ++i) { // 遍历所有灯条
            if (processed_indices.count(i)) { // 如果该矩形已处理，跳过
                continue;
            }

            const Light& light = lights[i]; // 取出当前灯条

            for (size_t j = 0; j < lights_count; ++j) {
                if (j != i && !processed_indices.count(j) && lights[j].color == light.color) { // 如果找到接近的灯条
                    int type;
                    float height;
                    std::tie(type, height) = is_close(light, lights[j]); // 调用 is_close 函数
                    // 获取类型 // 获取高度
                    std::cout << "type: " << type << ", height: " << height << std::endl;
                    if (type >= 0) { // 判断高度是否有效
                        Armor armor(light, lights[j], height, type); // 创建装甲板对象
                        armors_found.push_back(armor); // 添加装甲板到列表
                        processed_indices.insert(i); // 将已处理的矩形索引添加到 processed_indices 中
                        processed_indices.insert(j);
                    }
                }
            }
        }

        armors = armors_found; // 更新类成员
        return armors; // 返回装甲板列表
    }

    std::vector<Armor_info> id_armor() {
        std::vector<Armor_info> info_found; // 创建装甲板信息列表
        int img_height = img.rows; // 获取图像高度
        int img_width = img.cols; // 获取图像宽度

        for (const auto& armor : armors) { // 遍历所有装甲板
            cv::Point2f center = armor.center; // 获取装甲板中心

            // 计算中心坐标并进行反转
            float center_x = center.x - (img_width / 2);
            float center_y = -center.y + (img_height / 2); // 反转 y 坐标

            // 创建 Armor_info 实例并填充数据
            Armor_info info;
            info.class_id = armor.type_class(); // 获取装甲板类别 ID
            info.height = armor.height; // 获取装甲板高度
            info.cx = center_x; // 设置中心 x 坐标
            info.cy = center_y; // 设置中心 y 坐标
            info_found.push_back(info); // 将 Armor_info 实例添加到列表
        }

        armors_info = info_found; // 更新类成员
        return armors_info; // 返回装甲板信息列表
    }

    cv::Mat draw_lights(cv::Mat img_draw) { // 绘制灯条的函数
        for (const auto& light : lights) { // 遍历灯条
            if (light.color == 0) { // 如果颜色为红色
            // 绘制直线
            cv::line(img_draw, light.up, light.down, cv::Scalar(0, 100, 255), 1); 
            // 绘制中心点
            cv::circle(img_draw, cv::Point(static_cast<int>(light.cx), static_cast<int>(light.cy)), 1, cv::Scalar(255, 0, 0), -1); // 1是半径，-1表示填充  
            }
            else if (light.color == 1) { // 如果颜色为蓝色
                cv::line(img_draw, light.up, light.down, cv::Scalar(200, 71, 90), 1);
                cv::circle(img_draw, cv::Point(static_cast<int>(light.cx), static_cast<int>(light.cy)), 1, cv::Scalar(0, 0, 255), -1);
            }
        }      
        return img_draw; // 返回绘制后的图像
    }

    cv::Mat draw_armors(cv::Mat img_draw) { // 绘制装甲板的函数
        for (const auto& armor : armors) { // 遍历装甲板
            int img_height = img_draw.rows; // 图像高度
            int img_width = img_draw.cols; // 图像宽度
            // 获取中心点坐标
            cv::Point center = armor.center;
            // 坐标转换
            int center_x = static_cast<int>(center.x - (img_width / 2));
            int center_y = static_cast<int>(-(center.y - (img_height / 2))); // y轴反转

            if (armor.color == 0) { // 如果颜色为红色
                // 绘制装甲板的上下光条
                cv::line(img_draw, armor.light1_up, armor.light2_down, cv::Scalar(128, 0, 128), 1);
                cv::line(img_draw, armor.light2_up, armor.light1_down, cv::Scalar(128, 0, 128), 1);
                // 在图像上标记坐标
                cv::putText(img_draw, "(" + std::to_string(center_x) + ", " + std::to_string(center_y) + ")", 
                        center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(120, 255, 255), 2); // 绘制文本
            }
            else if (armor.color == 1){ // 如果颜色为蓝色
                // 绘制装甲板的上下光条
                cv::line(img_draw, armor.light1_up, armor.light2_down, cv::Scalar(255, 255, 0), 1);
                cv::line(img_draw, armor.light2_up, armor.light1_down, cv::Scalar(255, 255, 0), 1);
                // 在图像上标记坐标
                cv::putText(img_draw, "(" + std::to_string(center_x) + ", " + std::to_string(center_y) + ")", 
                        center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(120, 255, 255), 2); // 绘制文本
            }
        }
        return img_draw; // 返回绘制后的图像
    }

    cv::Mat draw_img() {
        cv::Mat img_draw = img.clone(); // 复制原始图像
        img_draw = draw_armors(img_draw); // 绘制装甲板
        img_drawn = draw_lights(img_draw); // 绘制灯条
        return img_drawn; // 返回绘制后的图像
    }
    // 显示函数
    std::tuple<cv::Mat, cv::Mat> display() {
        if (display_mode == 1) {
            return std::make_tuple(img_binary, cv::Mat()); // 返回二值图像和空图像
        } else if (display_mode == 2) {
            img_drawn = draw_img(); // 绘制图像
            return std::make_tuple(img_binary, img_drawn); // 返回二值图像和绘制后的图像
        } else if (display_mode == 0) {
            return std::make_tuple(cv::Mat(), cv::Mat()); // 返回两个空图像
        } else {
            std::cerr << "Invalid display mode" << std::endl;
            return std::make_tuple(cv::Mat(), cv::Mat()); // 返回两个空图像
        }
    }

    std::vector<Armor_info> detect_armors(const cv::Mat& img_input){
        img_binary = process(img_input);
        lights = find_lights(img_binary);
        armors = is_armor(lights);
        armors_info = id_armor();
        return armors_info;
    }

};

int main() {
    // 创建 light_params 对象并初始化
    int light_area_min = 5;
    int light_angle_min = -35;
    int light_angle_max = 35;
    float light_red_ratio = 1.0;
    float light_blue_ratio = 1.0;
    int cy_tol = 5;
    int height_tol = 20; 
    int light_angle_tol = 7;
    float vertical_discretization = 10;
    float height_multiplier = 2.7;
    Light_params light_params = {
        light_area_min, 
        light_angle_min, 
        light_angle_max, 
        light_red_ratio, 
        light_blue_ratio, 
        cy_tol,
        height_tol, 
        light_angle_tol, 
        vertical_discretization, 
        height_multiplier};

    //模式参数字典
    int detect_color =  2;  // 颜色参数 0: 识别红色装甲板, 1: 识别蓝色装甲板, 2: 识别全部装甲板
    int display_mode = 0; // 显示模式 0: 不显示, 1: 显示二值化图, 2: 显示二值化图和结果图像
    // 图像参数字典
    int binary_val = 225;
    ArmorDetector detector(detect_color, display_mode, binary_val, light_params); // 创建 ArmorDetector 对象

    // 读取输入图像
    cv::Mat input_image = cv::imread("./src/rm_opencv_aim/test/b.jpg");
    if (input_image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1; // 返回错误码
    }
    cv::Mat img_draw;
    std::vector<Armor_info> info = detector.detect_armors(input_image);
    
    for (const auto& inf : info) {
        // 处理找到的灯条（例如，输出数量）
        std::cout << "找到的armors: " << inf.cx << "," << inf.cy << std::endl;
    }
    //创建窗口并显示图像
    cv::namedWindow("Input Image", cv::WINDOW_AUTOSIZE); // 创建窗口
    cv::imshow("Input Image", input_image); // 显示输入图像

    cv::namedWindow("Binary Image", cv::WINDOW_AUTOSIZE); // 创建窗口
    cv::imshow("Binary Image", img_draw); // 显示二值化图像

    // 等待用户按键
    cv::waitKey(0); // 等待任意按键

    // 关闭所有窗口
    cv::destroyAllWindows();
    return 0;
}