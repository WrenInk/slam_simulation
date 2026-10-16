#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$SLAM_STUDY_ROOT"
OMNI_RUN_DIR="artifacts/omni_navigation_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)"
mkdir -p "$OMNI_RUN_DIR"
printf '%s\n' '10 m × 10 m 障碍场景：麦克纳姆全向小车 + MID360 + FAST-LIO2 + Nav2 Omni。' \
  '等待 RViz Navigation: active，使用 Nav2 Goal 在空闲区拖出目标及朝向。' \
  '地图原点约为起点；+X 向车头前方，+Y 向车身左侧。' \
  '首个横移目标可选地图 (0, 1.6)，箭头保持指向 +X。' \
  '关闭 MuJoCo 窗口或在本终端按 Ctrl+C 退出。' \
  "日志：$SLAM_STUDY_ROOT/$OMNI_RUN_DIR/launch.log"
ros2 launch nav_stack omni_navigation.launch.py output_dir:="$SLAM_STUDY_ROOT/$OMNI_RUN_DIR" "$@" \
  2>&1 | tee "$OMNI_RUN_DIR/launch.log"
