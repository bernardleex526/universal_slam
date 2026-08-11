#!/bin/bash
# ============================================================
# 将 FAST_LIO_ROS2 的 livox_ros_driver2 依赖改为可选 (WITH_LIVOX_SUPPORT)
# 背景: v1 只使用机械雷达（RoboSense 等标准 PointCloud2 输入），
#       livox_ros_driver2 不在 ROS apt 源中，无条件依赖会阻塞构建。
# 用法: 在 workspace 根目录（含 src/FAST_LIO_ROS2，由 vcs import 拉取）运行
# 说明: 幂等——已应用则跳过；重复运行安全。
# ============================================================
set -e
ROOT="src/FAST_LIO_ROS2"
if [ ! -f "$ROOT/CMakeLists.txt" ]; then
  echo "错误: 未找到 $ROOT —— 请先运行 setup_workspace.sh (vcs import)，再在 workspace 根目录运行本脚本"
  exit 1
fi

python3 - "$ROOT" <<'PYEOF'
import sys
root = sys.argv[1]

def load(rel):
    with open(f"{root}/{rel}", encoding='utf-8') as f:
        return f.read()

def save(rel, s):
    with open(f"{root}/{rel}", 'w', encoding='utf-8') as f:
        f.write(s)

def patch(rel, subs):
    """替换式补丁：new 已存在→跳过；old 存在→替换；否则失败。"""
    s = load(rel)
    for old, new in subs:
        if new in s:
            print(f"skip (already patched): {rel}")
        elif old in s:
            s = s.replace(old, new, 1)
            print(f"patched: {rel}")
        else:
            raise SystemExit(f"PATCH FAILED: 片段未找到且未应用 in {rel}:\n---\n{old[:200]}\n---")
    save(rel, s)

def remove(rel, olds):
    """删除式补丁：old 不存在→已删除跳过；存在→删除。"""
    s = load(rel)
    for old in olds:
        if old in s:
            s = s.replace(old, "", 1)
            print(f"removed: {rel}")
        else:
            print(f"skip (already removed): {rel}")
    save(rel, s)

def wrap_def(rel, sig, guard="#ifdef WITH_LIVOX_SUPPORT"):
    """函数定义级包裹：从签名到下一个 Preprocess:: 方法定义前的 } 整体加守卫。"""
    s = load(rel)
    marked = f"{guard}\n{sig}"
    if marked in s:
        print(f"skip (already patched): {rel} ({sig.split('(')[0].strip()})")
        return
    i = s.index(sig)
    j = s.index("\n}\n\nvoid Preprocess::", i)
    s = s[:i] + f"{guard}\n" + s[i:j+2] + f"\n#endif" + s[j+2:]
    save(rel, s)
    print(f"patched: {rel} ({sig.split('(')[0].strip()})")

# 1) CMakeLists.txt: livox 改为可选
patch("CMakeLists.txt", [
    ("find_package(livox_ros_driver2 REQUIRED)\n",
     "option(WITH_LIVOX_SUPPORT \"Build with Livox lidar support (requires livox_ros_driver2)\" OFF)\n"
     "if(WITH_LIVOX_SUPPORT)\n"
     "  find_package(livox_ros_driver2 REQUIRED)\n"
     "endif()\n"),
    ("ament_target_dependencies(fastlio_mapping ${dependencies})",
     "if(WITH_LIVOX_SUPPORT)\n"
     "  list(APPEND dependencies livox_ros_driver2)\n"
     "  target_compile_definitions(fastlio_mapping PRIVATE WITH_LIVOX_SUPPORT)\n"
     "endif()\n"
     "ament_target_dependencies(fastlio_mapping ${dependencies})"),
])
remove("CMakeLists.txt", ["  pcl_conversions\n  livox_ros_driver2\n"])

# 2) package.xml: 移除 livox 依赖声明
remove("package.xml", ["  <depend>livox_ros_driver2</depend>\n"])

# 3) src/preprocess.h: include + 成员函数声明加保护
patch("src/preprocess.h", [
    ("#include <livox_ros_driver2/msg/custom_msg.hpp>\n",
     "#ifdef WITH_LIVOX_SUPPORT\n"
     "#include <livox_ros_driver2/msg/custom_msg.hpp>\n"
     "#endif\n"),
    ("  void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);\n",
     "#ifdef WITH_LIVOX_SUPPORT\n"
     "  void process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out);\n"
     "#endif\n"),
    ("  void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);\n",
     "#ifdef WITH_LIVOX_SUPPORT\n"
     "  void avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg);\n"
     "#endif\n"),
])

# 4) src/laserMapping.cpp: include / 回调 / 订阅 / 成员声明加保护
patch("src/laserMapping.cpp", [
    ("#include <livox_ros_driver2/msg/custom_msg.hpp>\n",
     "#ifdef WITH_LIVOX_SUPPORT\n"
     "#include <livox_ros_driver2/msg/custom_msg.hpp>\n"
     "#endif\n"),
    ("void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) \n",
     "#ifdef WITH_LIVOX_SUPPORT\n"
     "void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) \n"),
    ("    sig_buffer.notify_all();\n}\n\nvoid imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)",
     "    sig_buffer.notify_all();\n}\n#endif\n\nvoid imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)"),
    ("""        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }""",
     """#ifdef WITH_LIVOX_SUPPORT
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 20, livox_pcl_cbk);
        }
        else
#endif
        {
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);
        }"""),
    ("    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;\n",
     "#ifdef WITH_LIVOX_SUPPORT\n"
     "    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;\n"
     "#endif\n"),
])

# 5) src/preprocess.cpp: 两个 livox 函数定义加保护
wrap_def("src/preprocess.cpp",
         "void Preprocess::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg, PointCloudXYZI::Ptr& pcl_out)")
wrap_def("src/preprocess.cpp",
         "void Preprocess::avia_handler(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg)")

print("OK: livox_ros_driver2 依赖已可选化 (WITH_LIVOX_SUPPORT=OFF 默认)")
PYEOF
