cmake_minimum_required(VERSION 3.8)
project(universal_slam_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 纯 launch 包：安装 launch 目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
