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

# ---- launch 契约测试（use_adapter 接线 / typed 参数对拍 / fail-fast）----
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_universal_slam_launch test/test_universal_slam_launch.py)
endif()

ament_package()
