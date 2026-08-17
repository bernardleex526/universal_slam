#!/bin/bash
# ============================================================
# save_map.sh —— 保存当前 FAST-LIO 建图结果，并可选转换为 Nav2 栅格地图
#
# 用法:
#   ./scripts/save_map.sh [map_name]
#
# 环境变量:
#   PCD_FILE   FAST-LIO 保存的 PCD 路径（默认 ~/.ros/universal_slam_map.pcd）
#   OUT_DIR    栅格地图输出目录（默认 ~/.ros）
#   SKIP_PCD2PGM  设为 1 可跳过 pcd2pgm 转换，只保存 PCD
#
# 依赖:
#   - FAST-LIO2 正在运行，且提供 /map_save 服务
#   - 可选: pcd2pgm 已编译并 source，用于 PCD -> PGM
# ============================================================
set -e

MAP_NAME="${1:-map}"
PCD_FILE="${PCD_FILE:-$HOME/.ros/universal_slam_map.pcd}"
OUT_DIR="${OUT_DIR:-$HOME/.ros}"
SKIP_PCD2PGM="${SKIP_PCD2PGM:-0}"

echo "[save_map] 触发 FAST-LIO 保存 PCD ..."
ros2 service call /map_save std_srvs/srv/Trigger {}

if [ ! -f "$PCD_FILE" ]; then
  echo "[save_map] 未找到 $PCD_FILE"
  echo "[save_map] 如果 FAST-LIO 的 map_file_path 不同，请通过 PCD_FILE 环境变量指定。"
  exit 1
fi

echo "[save_map] PCD 已保存: $PCD_FILE ($(du -h "$PCD_FILE" | cut -f1))"

if [ "$SKIP_PCD2PGM" = "1" ]; then
  echo "[save_map] 跳过 pcd2pgm 转换。"
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[save_map] 未找到 ros2，跳过栅格转换。"
  exit 0
fi

if ! ros2 pkg prefix pcd2pgm >/dev/null 2>&1; then
  echo "[save_map] 未检测到 pcd2pgm 包，跳过栅格转换。"
  echo "[save_map] 如需转换，请先编译并 source pcd2pgm 工作空间。"
  exit 0
fi

echo "[save_map] 启动 pcd2pgm 并生成栅格地图 ..."
TMP_YAML="$(mktemp /tmp/pcd2pgm_XXXXXX.yaml)"
cat > "$TMP_YAML" <<YAML
pcd2pgm:
  ros__parameters:
    pcd_file: "$PCD_FILE"
    odom_to_lidar_odom: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    flag_pass_through: true
    map_resolution: 0.05
    map_topic_name: map
    thre_radius: 0.1
    thre_z_max: 0.35
    thre_z_min: -0.04
    thres_point_count: 10
YAML

ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file "$TMP_YAML" &
PCD2PGM_PID=$!
cleanup() {
  kill "$PCD2PGM_PID" 2>/dev/null || true
  rm -f "$TMP_YAML"
}
trap cleanup EXIT

# 等待 map 话题出现（最多 30 秒）
for i in $(seq 1 30); do
  if ros2 topic info /map --quiet >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

mkdir -p "$OUT_DIR"
ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$MAP_NAME"
echo "[save_map] 栅格地图已保存: ${OUT_DIR}/${MAP_NAME}.pgm / .yaml"

cleanup
trap - EXIT
