cmake_minimum_required(VERSION 3.8)
project(robot_configs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 纯数据包：仅安装 config 目录
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
