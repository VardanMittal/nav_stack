FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install essential dev tools
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    nano \
    x11-apps \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
 && rm -rf /var/lib/apt/lists/*

# Source ROS automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspaces/nav_stack

CMD ["bash"]