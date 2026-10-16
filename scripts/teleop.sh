#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"
export PYTHONPATH="$SLAM_STUDY_ROOT/src/nav_stack:$PYTHONPATH"
python3 "$SLAM_STUDY_ROOT/scripts/teleop_keyboard.py" "$@"
