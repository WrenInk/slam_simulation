FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 安装编译依赖与 ROS 2 相关包
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-venv \
    python3-pip \
    libeigen3-dev \
    libpcl-dev \
    pcl-tools \
    libgl1 \
    libglfw3 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-sensor-msgs-py \
    ros-humble-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/slam_simulation
COPY . /workspace/slam_simulation

# 赋予执行权限并构建
RUN chmod +x scripts/*.sh && ./scripts/build.sh

# 默认配置环境
CMD ["/bin/bash", "--rcfile", "/workspace/slam_simulation/scripts/env.sh"]
