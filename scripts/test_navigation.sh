#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$SLAM_STUDY_ROOT"
export ROS_DOMAIN_ID="${TEST_ROS_DOMAIN_ID:-113}"
.venv/bin/python scripts/test_navigation.py
