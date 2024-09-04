import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件路径
    params_file = os.path.join(
        get_package_share_directory("mindvision_camera"), "config", "camera_params.yaml"
    )
    
    # 相机信息URL
    camera_info_url = "package://mindvision_camera/config/camera_info.yaml"

    # 创建launch描述
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(name="params_file", default_value=params_file),
        DeclareLaunchArgument(name="camera_info_url", default_value=camera_info_url),
        DeclareLaunchArgument(name="use_sensor_data_qos", default_value="false"),
        
        # 启动armor_detector_node
        Node(
            package="rm_serial_python",
            executable="rm_serial_node",
            output="screen",
            emulate_tty=True,
        ),        

        # 启动armor_detector_node
        Node(
            package="rm_yolo_aim",
            executable="armor_detector_node",
            output="screen",
            emulate_tty=True,
        ),
        
        # 新增启动armor_tracker_node
        Node(
            package="rm_yolo_aim",
            executable="armor_tracker_node",
            output="screen",
            emulate_tty=True,
        ),
        
        # 启动mindvision_camera_node
        Node(
            package="mindvision_camera",
            executable="mindvision_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                },
            ],
        ),
    ])