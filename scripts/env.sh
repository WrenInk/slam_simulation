# Source this file from bash: source scripts/env.sh
export SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [ -f "$SLAM_STUDY_ROOT/install/setup.bash" ]; then source "$SLAM_STUDY_ROOT/install/setup.bash"; fi
export PYTHONPATH="$SLAM_STUDY_ROOT/.venv/lib/python3.10/site-packages:$SLAM_STUDY_ROOT/src/nav_stack:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$SLAM_STUDY_ROOT/.deps/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
