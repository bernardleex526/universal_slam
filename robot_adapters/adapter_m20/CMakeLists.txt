cmake_minimum_required(VERSION 3.8)
project(adapter_m20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# nlohmann/json: Ubuntu 安装 sudo apt install nlohmann-json3-dev（rosdep key: nlohmann-json-dev）
find_package(nlohmann_json REQUIRED)

# ============================================================
# cmd_vel → UDP JSON 轴指令桥
# ============================================================
add_executable(m20_cmd_vel_bridge src/m20_cmd_vel_bridge.cpp)
ament_target_dependencies(m20_cmd_vel_bridge rclcpp sensor_msgs geometry_msgs std_msgs)
target_link_libraries(m20_cmd_vel_bridge nlohmann_json::nlohmann_json)

# ============================================================
# 话题桥（/IMU、/LIDAR/POINTS → /imu_raw、/points_raw）
# ============================================================
add_executable(m20_topic_bridge src/m20_topic_bridge.cpp)
ament_target_dependencies(m20_topic_bridge rclcpp sensor_msgs)

install(TARGETS
  m20_cmd_vel_bridge
  m20_topic_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
