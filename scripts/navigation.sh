#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"
cd "$SLAM_STUDY_ROOT"

DEFAULT_PCD="$SLAM_STUDY_ROOT/maps/map.pcd"
DEFAULT_YAML="$SLAM_STUDY_ROOT/maps/map.yaml"

# If maps/map.pcd doesn't exist yet, seed it from the latest verified map in artifacts
if [ ! -f "$DEFAULT_PCD" ]; then
    mkdir -p "$SLAM_STUDY_ROOT/maps"
    LATEST_ARTIFACT_PCD=$(find "$SLAM_STUDY_ROOT/artifacts" -name "map.pcd" -print 2>/dev/null | sort | tail -n 1 || true)
    if [ -n "$LATEST_ARTIFACT_PCD" ] && [ -f "$LATEST_ARTIFACT_PCD" ]; then
        echo "[提示] 检测到 maps/map.pcd 尚未建立，自动使用现有建图结果："
        echo "       $LATEST_ARTIFACT_PCD -> maps/"
        cp -f "$LATEST_ARTIFACT_PCD" "$DEFAULT_PCD"
        ARTIFACT_DIR="$(dirname "$LATEST_ARTIFACT_PCD")"
        if [ -f "$ARTIFACT_DIR/map.yaml" ]; then cp -f "$ARTIFACT_DIR/map.yaml" "$DEFAULT_YAML"; fi
        if [ -f "$ARTIFACT_DIR/map.pgm" ]; then cp -f "$ARTIFACT_DIR/map.pgm" "$SLAM_STUDY_ROOT/maps/map.pgm"; fi
    else
        echo "[错误] 未找到任何可用先验地图 (maps/map.pcd)。请先运行建图模式: ./scripts/mapping.sh"
        exit 1
    fi
fi

NAV_RUN_DIR="artifacts/navigation_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
mkdir -p "$NAV_RUN_DIR"

echo "=========================================================="
echo "      MID360 全向轮底盘先验地图导航模式 (Navigation)      "
echo "=========================================================="
echo " 先验 PCD 地图: $DEFAULT_PCD"
echo " 静态栅格地图: $DEFAULT_YAML"
echo " 导航日志目录: $NAV_RUN_DIR"
echo ""
echo " 操作提示："
echo "   1. RViz 中显示先验点云与地图，重定位节点将自动完成 ICP 初始对齐。"
echo "   2. 若机器人位置有偏差，可在 RViz 点击 '2D Pose Estimate' 重新指派初始位姿。"
echo "   3. 待 Nav2 启动完成 (Navigation: active) 后，使用 'Nav2 Goal' 点击目标位置和朝向！"
echo "   4. 终端按 Ctrl+C 安全退出。"
echo "=========================================================="

ros2 launch nav_stack navigation.launch.py \
    map_pcd:="$DEFAULT_PCD" \
    map_yaml:="$DEFAULT_YAML" \
    output_dir:="$NAV_RUN_DIR" \
    "$@"
