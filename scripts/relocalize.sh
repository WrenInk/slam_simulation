#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"
cd "$SLAM_STUDY_ROOT"

source "$SLAM_STUDY_ROOT/scripts/prior_map_args.sh"
prepare_prior_map_args "$@"
if "$PRIOR_MAP_INSPECT"; then
    exec ros2 launch nav_stack relocalize.launch.py "$@"
fi

RUN_DIR="artifacts/relocalize_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"

echo "=========================================================="
echo "         MID360 手动重定位演示模式 (Manual Relocalization)         "
echo "=========================================================="
echo " 先验 PCD 地图: $PRIOR_MAP_PCD"
echo " 静态栅格地图: $PRIOR_MAP_YAML（缺失时由点云生成）"
echo " 演示日志目录: $RUN_DIR"
echo ""
echo " 操作演示指引："
echo "   1. [观察初始错位]：底盘默认在非原点位置出生 (x=-2.0, y=0.0)，"
echo "      RViz 中仅展示 2D 占据栅格地图与青色实时扫描点云，两者此时呈现明显错位！"
echo "   2. [手动触发重定位]：点击 RViz 顶栏 '2D Pose Estimate' 工具，"
echo "      在地图上机器人实际所在大致区域点击并拖动出朝向箭头。"
echo "   3. [观察匹配结果]：ICP 尝试对齐点云；结果取决于初始位姿与扫描重叠程度。"
echo "   4. 终端按 Ctrl+C 安全退出。"
echo "=========================================================="

ros2 launch nav_stack relocalize.launch.py \
    map_pcd:="$PRIOR_MAP_PCD" \
    map_yaml:="$PRIOR_MAP_YAML" \
    output_dir:="$RUN_DIR" \
    "${PRIOR_MAP_ROS_ARGS[@]}"
