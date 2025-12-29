FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Install essential dev tools
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    nano \
    iputils-ping \
    net-tools \
    mesa-utils \
    x11-apps \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # --- Robot Body & Control ---
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    # --- Simulation (The Fix) ---
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gz-ros2-control \
    ros-humble-ign-ros2-control\
    # ^^^ Added this! Often needed for compatibility with Fortress
    # --- The "Neuro-Nav" Intelligence Stack ---
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-msgs \
 && rm -rf /var/lib/apt/lists/*

ENV IGN_IP=127.0.0.1
ENV IGN_TRANSPORT_TOPIC_STATISTICS=0

# Source ROS automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

WORKDIR /workspaces/nav_stack

CMD ["bash"]