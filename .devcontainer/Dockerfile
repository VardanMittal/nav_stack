FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# -------------------------
# Base tools
# -------------------------
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    nano \
    curl \
    lsb-release \
    iputils-ping \
    net-tools \
    mesa-utils \
    x11-apps \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# ROS Control + Navigation
# -------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Gazebo Fortress (ROS-GZ way)
# -------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# -------------------------
# Environment
# -------------------------
ENV IGN_IP=127.0.0.1
ENV IGN_TRANSPORT_TOPIC_STATISTICS=0

# Correct plugin paths
ENV IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib:/usr/lib
ENV LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

# -------------------------
# Auto-source ROS
# -------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspaces/nav_stack
CMD ["bash"]
