#!/bin/bash
# ============================================================
# save_map.sh —— 保存当前建图结果 (P1 尚未实现)
#
# fail-fast 占位：地图管线（D9）未实现，本脚本不以"成功"姿态退出。
#   1. 触发 FAST-LIO 的 map_save 服务保存 PCD（服务名以实测为准）
#   2. 将 PCD 经 pcd2pgm 转为 2D 栅格 (PGM + YAML)
#   3. 输出栅格作为 Nav2 全局代价地图 static layer 使用
# 未实现期间：universal_slam.launch.py 不发布 /map、static layer 保持禁用，
# 导航模式 = FAST-LIO2 持续建图 + 实时点云代价地图（见 README.md）。
# ============================================================
set -e

echo "[ERROR] save_map.sh 尚未实现 (P1)。当前系统不发布 /map，也没有静态地图可保存。"
echo "        详见 README.md「地图管线未实现」与 PLAN.md D9。"
exit 1
