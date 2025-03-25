cmake_minimum_required(VERSION 3.8)
project(rm_opencv_aim)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(jsoncpp REQUIRED)  # 添加 JSON 库依赖
find_package(rclcpp REQUIRED)  # 添加 ROS 2 rclcpp 依赖
find_package(OpenCV REQUIRED)   # 添加 OpenCV 依赖

# 添加可执行文件
add_executable(armor_detector_opencv src/armor_detector_opencv.cpp)

# 连接库
ament_target_dependencies(armor_detector_opencv rclcpp OpenCV)

# 安装可执行文件
install(TARGETS
  armor_detector_opencv
  DESTINATION lib/${PROJECT_NAME}
)

# 如果需要测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 下面的行跳过检查版权的 linter
  set(ament_cmake_copyright_FOUND TRUE)
  # 跳过 cpplint（仅在 git 仓库中工作）
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp OpenCV)  # 导出依赖项
ament_package()