from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 创建launch描述
    return LaunchDescription([
        
        # 启动 rm_serial_node
        Node(
            package="rm_serial_python",
            executable="rm_serial_node",
            output="screen",
            emulate_tty=True,
        ),        

        # 启动 armor_detector_node
        Node(
            package="rm_yolo_aim",
            executable="armor_detector_node",
            output="screen",
            emulate_tty=True,
        ),
        
        # 启动 armor_tracker_node
        Node(
            package="rm_yolo_aim",
            executable="armor_tracker_node",
            output="screen",
            emulate_tty=True,
        ),
        
        # 启动 v4l2_camera_node
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            output="screen",
            emulate_tty=True,
        ),

    ])