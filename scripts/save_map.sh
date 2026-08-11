#!/bin/bash
# ============================================================
# save_map.sh —— 保存当前建图结果 (P1 完善)
#
# TODO(P1):
#   1. 触发 FAST-LIO 的 map_save 服务保存 PCD
#      ros2 service call /pcl_map_save std_srvs/srv/Empty {}
#      （Ericsii/FAST_LIO_ROS2 服务名以实测为准）
#   2. 将 PCD 经 pcd2pgm (LihanChen2004) 转为 2D 栅格 (PGM + YAML)
#      - z 窗口 PassThrough: thre_z_min≈-0.04 / thre_z_max≈0.35 (参考值)
#      - 输出分辨率 0.05，帧对齐 camera_init 系（与 FAST-LIO 同起点部署）
#   3. 输出栅格作为 Nav2 全局代价地图 static layer 使用
#
# 本脚本当前为占位，P1 实现上述流程后再启用。
# ============================================================
set -e

echo "[TODO] save_map.sh 尚未实现 (P1)。详见 PLAN.md D9 地图保存与复用。"
exit 0
