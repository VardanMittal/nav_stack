FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install essential dev tools only
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
 && rm -rf /var/lib/apt/lists/*

# Source ROS automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Workspace directory (DevContainer will mount here)
WORKDIR /workspaces/nav_stack

CMD ["bash"]
