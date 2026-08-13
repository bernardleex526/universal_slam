cmake_minimum_required(VERSION 3.8)
project(adapter_sw01)

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
find_package(Threads REQUIRED)

# ============================================================
# SW01 专有 SDK (Daedalus, libASTRALL_SDK.so)
# - Ubuntu 部署时设置 ASTRALL_SDK_DIR 指向拷贝到本机的 SDK 目录:
#     colcon build --cmake-args -DASTRALL_SDK_DIR=/path/to/ASTRALL_SDK
#   SDK 布局参考（Windows 调研用）:
#     D:\璇玑\技术文档\ASTRALL_SDK_1.0.7\ASTRALL_SDK_1.0.7\C++\
#       ├─ include/interface.h
#       └─ lib/linux/{x86_64,ARM64}/libASTRALL_SDK.so(.1/.1.0.7)
# - 未设置 ASTRALL_SDK_DIR 时跳过主目标；P0 单测（纯函数）始终可构建，
#   便于先验证解析/数学逻辑（不依赖 SDK 头文件）。
# ============================================================
set(ASTRALL_SDK_DIR "" CACHE PATH "ASTRALL SDK (libASTRALL_SDK.so) 安装目录")

if(ASTRALL_SDK_DIR)
  message(STATUS "ASTRALL_SDK_DIR = ${ASTRALL_SDK_DIR}")

  # 平台化库路径（Linux x86_64 / ARM64）
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
    set(ASTRALL_SDK_LIB "${ASTRALL_SDK_DIR}/lib/linux/ARM64/libASTRALL_SDK.so")
  else()
    set(ASTRALL_SDK_LIB "${ASTRALL_SDK_DIR}/lib/linux/x86_64/libASTRALL_SDK.so")
  endif()
  message(STATUS "ASTRALL SDK lib = ${ASTRALL_SDK_LIB}")

  add_executable(sw01_adapter src/sw01_adapter.cpp)
  ament_target_dependencies(sw01_adapter rclcpp sensor_msgs geometry_msgs std_msgs)
  target_include_directories(sw01_adapter PRIVATE ${ASTRALL_SDK_DIR}/include include)
  target_link_libraries(sw01_adapter ${ASTRALL_SDK_LIB} Threads::Threads)

  install(TARGETS sw01_adapter
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "ASTRALL_SDK_DIR 未设置，跳过 sw01_adapter 目标。"
    "Ubuntu 部署时: colcon build --cmake-args -DASTRALL_SDK_DIR=/path/to/ASTRALL_SDK")
endif()

# 纯函数头 + launch（bringup 的 use_adapter:=true 需要）
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# ---- P0 单测（纯函数，不依赖 SDK，无 ASTRALL_SDK_DIR 也可构建运行）----
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cloud_packet test/test_cloud_packet.cpp)
  target_include_directories(test_cloud_packet PRIVATE include)

  ament_add_gtest(test_sw01_math test/test_sw01_math.cpp)
  target_include_directories(test_sw01_math PRIVATE include)
endif()

ament_package()
