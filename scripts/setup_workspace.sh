#!/bin/bash
# ============================================================
# 通用 SLAM 系统 —— 工作区初始化脚本 (P0)
#
# 使用前提（Ubuntu 22.04 + ROS2 Humble）：
#   source /opt/ros/humble/setup.bash
# 首次使用请先执行上面的 source，再运行本脚本。
# ============================================================
set -e

# 1) 检查 vcstool
if ! command -v vcs &> /dev/null; then
  echo "[ERROR] 未检测到 vcs (vcstool)，请先安装: sudo apt install python3-vcstool"
  exit 1
fi

# 2) 导入第三方依赖仓库 (FAST_LIO_ROS2 / pcd2pgm) 到 src/
mkdir -p src
vcs import src < universal_slam.repos

# 2.1) 应用第三方补丁
#      FAST_LIO_ROS2: livox_ros_driver2 依赖可选化（v1 只用机械雷达，apt 无此包）
bash scripts/patch_fastlio_livox.sh

# 3) 解析并安装系统依赖
rosdep install --from-paths src --ignore-src -r -y

# 4) 构建（symlink-install 便于 config/launch 热更新）
colcon build --symlink-install

echo "[OK] 构建完成。使用前请 source install/setup.bash"
