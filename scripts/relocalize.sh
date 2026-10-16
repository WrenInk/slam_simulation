#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"
cd "$SLAM_STUDY_ROOT"

DEFAULT_PCD="$SLAM_STUDY_ROOT/maps/map.pcd"
DEFAULT_YAML="$SLAM_STUDY_ROOT/maps/map.yaml"

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

RUN_DIR="artifacts/relocalize_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"

echo "=========================================================="
echo "         MID360 手动重定位演示模式 (Manual Relocalization)         "
echo "=========================================================="
echo " 先验 PCD 地图: $DEFAULT_PCD"
echo " 静态栅格地图: $DEFAULT_YAML"
echo " 演示日志目录: $RUN_DIR"
echo ""
echo " 操作演示指引："
echo "   1. [观察初始错位]：底盘默认在非原点位置出生 (x=-2.0, y=0.0)，"
echo "      RViz 中仅展示 2D 占据栅格地图与青色实时扫描点云，两者此时呈现明显错位！"
echo "   2. [手动触发重定位]：点击 RViz 顶栏 '2D Pose Estimate' 工具，"
echo "      在地图上机器人实际所在大致区域点击并拖动出朝向箭头。"
echo "   3. [观察点云吸附]：多角度粗精 ICP 将瞬间完成对齐，青色点云瞬间卡入栅格墙体中！"
echo "   4. 终端按 Ctrl+C 安全退出。"
echo "=========================================================="

ros2 launch nav_stack relocalize.launch.py \
    map_pcd:="$DEFAULT_PCD" \
    map_yaml:="$DEFAULT_YAML" \
    output_dir:="$RUN_DIR" \
    "$@"
