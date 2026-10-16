#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"
cd "$SLAM_STUDY_ROOT"

MAPPING_RUN_DIR="artifacts/mapping_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
MAP_TARGET_DIR="$SLAM_STUDY_ROOT/maps"
mkdir -p "$MAPPING_RUN_DIR" "$MAP_TARGET_DIR"

echo "=========================================================="
echo "         MID360 全向轮底盘建图模式 (Mapping Mode)         "
echo "=========================================================="
echo " 仿真与建图启动中..."
echo " 地图输出路径: $MAP_TARGET_DIR/ (map.pcd, map.yaml, map.pgm)"
echo " 运行日志备份: $MAPPING_RUN_DIR"
echo ""
echo " 提示: 另开一个终端运行键盘控制："
echo "   cd $SLAM_STUDY_ROOT && ./scripts/teleop.sh"
echo ""
echo " 建图完成后，在此终端按 Ctrl+C，系统将自动保存 PCD 和栅格地图！"
echo "=========================================================="

ros2 launch nav_stack mapping.launch.py \
    output_dir:="$MAPPING_RUN_DIR" \
    map_save_dir:="$MAP_TARGET_DIR" \
    "$@"
