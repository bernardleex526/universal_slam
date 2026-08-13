cmake_minimum_required(VERSION 3.8)
project(slam_nav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# P0: 纯配置包 —— 仅安装 config 与 launch，无源码目标
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# ---- launch 契约测试（typed 参数回归）----
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fastlio_launch test/test_fastlio_launch.py)
endif()

ament_package()
