#!/usr/bin/env bash
set -eo pipefail
SLAM_STUDY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$SLAM_STUDY_ROOT"
source /opt/ros/humble/setup.bash
if [ ! -x .venv/bin/python ]; then
    if command -v uv >/dev/null; then
        uv venv --system-site-packages --python /usr/bin/python3 .venv
    else
        /usr/bin/python3 -m venv --system-site-packages .venv
    fi
fi
if command -v uv >/dev/null; then
    uv pip install --python .venv/bin/python -r requirements.txt
else
    .venv/bin/python -m pip install -r requirements.txt
fi
cmake -S third_party/Livox-SDK2 -B build/livox_sdk -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$SLAM_STUDY_ROOT/.deps"
cmake --build build/livox_sdk -j2
cmake --install build/livox_sdk
export MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --base-paths src --executor sequential --symlink-install --cmake-args \
    -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DLIVOX_LIDAR_SDK_LIBRARY="$SLAM_STUDY_ROOT/.deps/lib/liblivox_lidar_sdk_shared.so" \
    -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$SLAM_STUDY_ROOT/.deps/include"
source scripts/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q tests
