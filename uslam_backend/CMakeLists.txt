cmake_minimum_required(VERSION 3.8)
project(uslam_backend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters kdtree registration)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

# Algorithm core: no ROS types, unit-tested on its own.
add_library(uslam_backend_core
  src/scan_context.cpp src/pose_graph.cpp src/registration.cpp src/map_io.cpp)
target_include_directories(uslam_backend_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
target_link_libraries(uslam_backend_core ${PCL_LIBRARIES} gtsam)
target_compile_definitions(uslam_backend_core PUBLIC ${PCL_DEFINITIONS})

add_executable(backend_node src/backend_node.cpp)
target_link_libraries(backend_node uslam_backend_core)
ament_target_dependencies(backend_node rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs std_srvs
  tf2 tf2_ros tf2_eigen pcl_conversions)

install(TARGETS backend_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS uslam_backend_core DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_scan_context test/test_scan_context.cpp)
  target_link_libraries(test_scan_context uslam_backend_core)
  ament_add_gtest(test_pose_graph test/test_pose_graph.cpp)
  target_link_libraries(test_pose_graph uslam_backend_core)
  ament_add_gtest(test_registration test/test_registration.cpp)
  target_link_libraries(test_registration uslam_backend_core)
  ament_add_gtest(test_map_io test/test_map_io.cpp)
  target_link_libraries(test_map_io uslam_backend_core)
  ament_add_pytest_test(test_backend_node test/test_backend_node.py TIMEOUT 60)
endif()

ament_package()
