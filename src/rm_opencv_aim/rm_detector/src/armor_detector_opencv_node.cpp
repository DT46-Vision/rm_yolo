#include <rclcpp/rclcpp.hpp> // 引入 ROS2 C++ 客户端库
#include <sensor_msgs/msg/image.hpp> // 引入图像消息类型
#include <sensor_msgs/msg/camera_info.hpp> // 引入相机信息消息类型
#include <cv_bridge/cv_bridge.h> // 引入 OpenCV 与 ROS 图像转换类
#include <opencv2/opencv.hpp> // 引入 OpenCV 图像处理库

// 引入自定义消息类型
#include "rm_interfaces/msg/armor_info.hpp" // 引入 ArmorInfo 消息类型
#include "rm_interfaces/msg/armors_msg.hpp" // 引入 ArmorsMsg 消息类型
#include "rm_interfaces/msg/serial_receive.hpp" // 引入 SerialReceive 消息类型
#include "rm_interfaces/msg/decision.hpp" // 引入 Decision 消息类型

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode() 
        : Node("armor_detector_opencv_node") { 
        // 创建 light_params 对象并初始化
        Light_params light_params = {
            5,    // light_area_min
            -35,  // light_angle_min
            35,   // light_angle_max
            1.0,  // light_red_ratio
            1.0,  // light_blue_ratio
            5,    // cy_tol
            10,   // height_tol
            7,    // light_angle_tol
            2.1,  // vertical_discretization
            2.7   // height_multiplier
        };

        // 模式参数字典
        int detect_color = 2;  // 颜色参数
        int display_mode = 2; // 显示模式
        int binary_val = 225;

        // 创建 ArmorDetector 对象并初始化
        detector_ = std::make_shared<ArmorDetector>(detect_color, display_mode, binary_val, light_params);
        
        // 创建参数服务
        this->declare_parameter<int>("light_area_min", 5);
        this->declare_parameter<int>("light_angle_min", -35);
        this->declare_parameter<int>("light_angle_max", 35);
        this->declare_parameter<float>("light_red_ratio", 1.0);
        this->declare_parameter<float>("light_blue_ratio", 1.0);
        this->declare_parameter<int>("cy_tol", 5);
        this->declare_parameter<int>("height_tol", 10);
        this->declare_parameter<int>("light_angle_tol", 7);
        this->declare_parameter<float>("vertical_discretization", 2.1);
        this->declare_parameter<float>("height_multiplier", 2.7);
        this->declare_parameter<int>("binary_val", 225);
        this->declare_parameter<int>("detect_color", 2);
        this->declare_parameter<int>("display_mode", 2);

        // 注册参数回调
        this->set_on_parameters_set_callback(
            std::bind(&ArmorDetectorNode::parameters_callback, this, std::placeholders::_1));
        
        // 创建图像消息订阅者
        sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ArmorDetectorNode::image_callback, this, std::placeholders::_1));
        
        // 创建相机信息订阅者
        sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", 10, std::bind(&ArmorDetectorNode::camera_info_callback, this, std::placeholders::_1));
        
        // 创建发布者
        publisher_armors_ = this->create_publisher<rm_interfaces::msg::ArmorsMsg>("/detector/armors_info", 10);
        publisher_armors_img_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/armors_img", 10); // 发布 armors_img
        publisher_bin_img_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/bin_img", 10); // 发布 bin_img

        RCLCPP_INFO(this->get_logger(), "Armor Detector Node has been started."); // 输出节点启动信息
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) { // 图像回调函数
        auto cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image; // 转换为 OpenCV 图像
        
        cv::Mat bin;
        cv::Mat drawn;

        std::vector<Armor_info> armors_info;

        // 使用 ArmorDetector 对象处理图像
        armors_info = detector_->detect_armors(cv_image); // 使用传入的 detector 对象
        std::tie(bin, drawn) = detector_->display();

        // 示例：发布装甲信息
        rm_interfaces::msg::ArmorsMsg armors_msg; // 创建 ArmorsMsg 消息
        armors_msg.header.stamp = this->get_clock()->now(); // 设置时间戳
        armors_msg.header.frame_id = "camera_frame"; // 设置帧 ID

        // 遍历 armors_info，将内容添加到 armors_msg 中
        for (const auto& armor : armors_info) { // 遍历装甲信息数组
            rm_interfaces::msg::ArmorInfo armor_info; // 创建 ArmorInfo 消息
            armor_info.height = armor.height; // 设置高度
            armor_info.class_id = armor.class_id; // 设置类别 ID
            armor_info.cx = armor.cx; // 设置中心 x 坐标
            armor_info.cy = armor.cy; // 设置中心 y 坐标
            armors_msg.armors.push_back(armor_info); // 将装甲信息添加到消息中
        }

        // 发布装甲信息
        publisher_armors_->publish(armors_msg); // 发布装甲信息
        RCLCPP_INFO(this->get_logger(), "Published armors information."); // 输出发布信息

        sensor_msgs::msg::Image bin_img_msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", bin).toImageMsg();
        publisher_bin_img_->publish(bin_img_msg); // 发布二值图像
        RCLCPP_INFO(this->get_logger(), "Published binary image."); // 输出二值图像发布信息

        // 发布图像消息
        sensor_msgs::msg::Image armors_img_msg = *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", drawn).toImageMsg();
        publisher_armors_img_->publish(armors_img_msg); // 发布装甲图像
        RCLCPP_INFO(this->get_logger(), "Published armors image."); // 输出图像发布信息

    }

    rclcpp::ParameterCallbackReturn parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
        for (const auto &param : parameters) {
            if (param.get_name() == "light_area_min") {
                detector_->update_light_area_min(param.as_int());
            } else if (param.get_name() == "light_angle_min") {
                detector_->update_light_angle_min(param.as_int());
            } else if (param.get_name() == "light_angle_max") {
                detector_->update_light_angle_max(param.as_int());
            } else if (param.get_name() == "light_red_ratio") {
                detector_->update_light_red_ratio(param.as_double());
            } else if (param.get_name() == "light_blue_ratio") {
                detector_->update_light_blue_ratio(param.as_double());
            } else if (param.get_name() == "cy_tol") {
                detector_->update_cy_tol(param.as_int());
            } else if (param.get_name() == "height_tol") {
                detector_->update_height_tol(param.as_int());
            } else if (param.get_name() == "light_angle_tol") {
                detector_->update_light_angle_tol(param.as_int());
            } else if (param.get_name() == "vertical_discretization") {
                detector_->update_vertical_discretization(param.as_double());
            } else if (param.get_name() == "height_multiplier") {
                detector_->update_height_multiplier(param.as_double());
            } else if (param.get_name() == "binary_val") {
                detector_->update_binary_val(param.as_int());
            } else if (param.get_name() == "detect_color") {
                detector_->update_detect_color(param.as_int());
            } else if (param.get_name() == "display_mode") {
                detector_->update_display_mode(param.as_int());
            }
        }
        return rclcpp::ParameterCallbackReturn::SUCCESS;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_; // 图像消息订阅者
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_; // 相机信息订阅者
    rclcpp::Publisher<rm_interfaces::msg::ArmorsMsg>::SharedPtr publisher_armors_; // 装甲信息发布者
    std::shared_ptr<ArmorDetector> detector_; // ArmorDetector 对象
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // 初始化 ROS2

    // 创建并运行 ArmorDetectorNode
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());

    rclcpp::shutdown(); // 关闭 ROS2
    return 0; // 返回成功
}