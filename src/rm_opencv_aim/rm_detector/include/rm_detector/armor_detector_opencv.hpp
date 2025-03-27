#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <tuple>
#include <set>

// 计算两个点之间的距离
double calculate_distance(const cv::Point2f& p1, const cv::Point2f& p2);

// 调整宽高和角度的函数
std::pair<cv::Size2f, double> adjust(const cv::Size2f& w_h, double angle);

// 将角度转换为斜率的函数
double angle_to_slope(double angle_degrees);

// 定义 Light 类
class Light {
public:
    int cx;          // 中心 x 坐标
    int cy;          // 中心 y 坐标
    double height;   // 灯条高度
    cv::Point2f up;  // 上端点
    cv::Point2f down;// 下端点
    double angle;    // 灯条角度
    int color;       // 灯条颜色

    Light(const cv::Point2f& up, const cv::Point2f& down, double angle, int color);
};

// 定义 Armor 类
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

    Armor(const Light& light1, const Light& light2, double height, int type);
    int type_class() const; // 类型分类函数
};

// 定义 Armor_info 结构体
struct Armor_info {
    float height; // 装甲板高度
    int class_id; // 装甲板类别 ID
    float cx; // 中心 x 坐标
    float cy; // 中心 y 坐标
};

// 定义 Light_params 结构体
struct Light_params {
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
};

// 定义 ArmorDetector 类
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

    ArmorDetector(int detect_color, int display_mode, int binary_val, const Light_params light_params);
    
    void update_light_area_min(int new_light_area_min);
    void update_light_angle_min(int new_light_angle_min);
    void update_light_angle_max(int new_light_angle_max);
    void update_light_red_ratio(float new_light_red_ratio);
    void update_light_blue_ratio(float new_light_blue_ratio);
    void update_cy_tol(int new_cy_tol);
    void update_height_tol(int new_height_tol);
    void update_light_angle_tol(int new_light_angle_tol);
    void update_vertical_discretization(float new_vertical_discretization);
    void update_height_multiplier(float new_height_multiplier);
    void update_binary_val(int new_binary_val);
    void update_detect_color(int new_color);
    void update_display_mode(int new_display_mode);
    cv::Mat process(const cv::Mat& img_input);
    std::vector<Light> find_lights(const cv::Mat& img_binary_input);
    std::pair<int, float> is_close(const Light& light1, const Light& light2);
    std::vector<Armor> is_armor(const std::vector<Light>& lights);
    std::vector<Armor_info> id_armor();
    cv::Mat draw_lights(cv::Mat img_draw);
    cv::Mat draw_armors(cv::Mat img_draw);
    cv::Mat draw_img();
    std::tuple<cv::Mat, cv::Mat> display();
    std::vector<Armor_info> detect_armors(const cv::Mat& img_input);
};

#endif // ARMOR_DETECTOR_HPP