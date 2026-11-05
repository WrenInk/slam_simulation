#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"
cd "$SLAM_STUDY_ROOT"

source "$SLAM_STUDY_ROOT/scripts/prior_map_args.sh"
prepare_prior_map_args "$@"
if "$PRIOR_MAP_INSPECT"; then
    exec ros2 launch nav_stack navigation.launch.py "$@"
fi

NAV_RUN_DIR="artifacts/navigation_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
mkdir -p "$NAV_RUN_DIR"

echo "=========================================================="
echo "      MID360 全向轮底盘先验地图导航模式 (Navigation)      "
echo "=========================================================="
echo " 先验 PCD 地图: $PRIOR_MAP_PCD"
echo " 静态栅格地图: $PRIOR_MAP_YAML（缺失时由点云生成）"
echo " 导航日志目录: $NAV_RUN_DIR"
echo ""
echo " 操作提示："
echo "   1. RViz 中显示先验点云与地图，重定位节点尝试通过 ICP 完成初始对齐。"
echo "   2. 若机器人位置有偏差，可在 RViz 点击 '2D Pose Estimate' 重新指派初始位姿。"
echo "   3. 待 Nav2 启动完成 (Navigation: active) 后，使用 'Nav2 Goal' 点击目标位置和朝向！"
echo "   4. 终端按 Ctrl+C 安全退出。"
echo "=========================================================="

ros2 launch nav_stack navigation.launch.py \
    map_pcd:="$PRIOR_MAP_PCD" \
    map_yaml:="$PRIOR_MAP_YAML" \
    output_dir:="$NAV_RUN_DIR" \
    "${PRIOR_MAP_ROS_ARGS[@]}"
