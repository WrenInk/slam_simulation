# Sources and licenses

This is an extraction of the working obstacle-arena integration. Vendored sources
already contain the ROS 2 adaptations; do not overwrite them with upstream sources.
Pinned upstream revisions are in sources.lock.json. FAST-LIO modifications are
also recorded in patches/fast_lio_ros2.patch.

- FAST-LIO2: https://github.com/hku-mars/FAST_LIO (GPL-2.0).
- Livox driver: https://github.com/Livox-SDK/livox_ros_driver2; ROS 2 manifest selected.
- Livox SDK2: https://github.com/Livox-SDK/Livox-SDK2, built locally into .deps.
- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM, revision
  08af3f32f01725372d4269838dc44c19c6d9e76b (BSD-3-Clause).
  pose_graph.py is an algorithm port of keyframe selection, odometry/loop factors,
  radius/time loop candidates and ICP validation, using GTSAM iSAM2. It is not the
  unchanged LIO-SAM executable. FAST-LIO supplies inertial odometry; no separate
  LIO-SAM IMU preintegration or GPS factors are included.
- Nav2: https://github.com/ros-navigation/navigation2 (installed ROS Humble packages,
  Apache-2.0); SmacPlanner2D and MPPI Omni.
- GTSAM: https://github.com/borglab/gtsam, Python wheel 4.2.
- MuJoCo: https://github.com/google-deepmind/mujoco, Python wheel 3.3.7.

License notices are retained in LICENSES and vendor directories. The integration
license does not replace upstream licenses. The robot and obstacle arena are
primitive geometry models. MID360 scanning and mecanum traction are simplified
simulation models, not calibrated hardware replicas. Simulator ground truth is
used only for evaluation, not SLAM or navigation feedback.

MID-360 model update:
- The exterior mesh is converted from Livox's official STEP model at
  https://www.livoxtech.com/mid-360/downloads. See third_party/mid360_official/README.md
  and sources.lock.json for the original download and checksum.
- Scan directions are converted from the official MIT-licensed
  https://github.com/Livox-SDK/livox_laser_simulation mid360.csv, preserving the
  plugin's Euler angle convention. The upstream ROS 1 / Gazebo plugin is retained
  for reference, not compiled into this ROS 2 / MuJoCo workspace.
- MuJoCo raycasting, simulated IMU, timestamp batching, range/noise approximation
  and reduced ray budget remain integration code; this is not an unmodified
  official simulator or a calibrated sensor digital twin.
