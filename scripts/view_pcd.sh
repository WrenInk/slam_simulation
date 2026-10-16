#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SLAM_STUDY_ROOT/scripts/env.sh"

"$SLAM_STUDY_ROOT/.venv/bin/python" "$SLAM_STUDY_ROOT/scripts/view_pcd.py" "$@"
