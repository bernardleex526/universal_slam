cmake_minimum_required(VERSION 3.8)
project(slam_nav_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(PROGRAMS scripts/odometry_adapter.py scripts/obstacle_filter.py scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME})

# Robot-agnostic algorithm package: config + launch + Python nodes, no robot dependency.
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fastlio_launch test/test_fastlio_launch.py)
  ament_add_pytest_test(test_nav2_config test/test_nav2_config.py)
  ament_add_pytest_test(test_odometry_adapter test/test_odometry_adapter.py)
  ament_add_pytest_test(test_obstacle_filter test/test_obstacle_filter.py)
  ament_add_pytest_test(test_terrain test/test_terrain.py TIMEOUT 120)
  ament_add_pytest_test(test_save_map_grid test/test_save_map_grid.py)
  ament_add_pytest_test(test_empty_map_save test/test_empty_map_save.py TIMEOUT 20)
  ament_add_pytest_test(test_fastlio_gravity_alignment test/test_fastlio_gravity_alignment.py TIMEOUT 45)
endif()

ament_package()
