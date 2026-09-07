cmake_minimum_required(VERSION 3.8)
project(slam_nav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/odometry_adapter.py scripts/obstacle_filter.py DESTINATION lib/${PROJECT_NAME})

# P0: 纯配置包 —— 仅安装 config 与 launch，无源码目标
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# ---- launch 契约测试（typed 参数回归）----
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fastlio_launch test/test_fastlio_launch.py)
  ament_add_pytest_test(test_nav2_config test/test_nav2_config.py)
  ament_add_pytest_test(test_odometry_adapter test/test_odometry_adapter.py)
  ament_add_pytest_test(test_empty_map_save test/test_empty_map_save.py TIMEOUT 20)
endif()

ament_package()
