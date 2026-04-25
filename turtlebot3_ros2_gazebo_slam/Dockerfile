FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=:1
ENV ROS_DOMAIN_ID=0

# Dev tools
RUN apt-get update && apt-get install -y \
    build-essential cmake git curl \
    python3-colcon-common-extensions \
    python3-rosdep python3-pip \
    && rm -rf /var/lib/apt/lists/*

# TurtleBot3 + Gazebo (ROS-GZ)
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# noVNC + X server stack
RUN apt-get update && apt-get install -y \
    xvfb x11vnc novnc websockify fluxbox xdotool xterm \
    && rm -rf /var/lib/apt/lists/*

# Workspace
RUN mkdir -p /ros2_ws/packages
WORKDIR /ros2_ws

# Scripts
COPY entrypoint.sh /entrypoint.sh
COPY start.sh /start.sh

RUN chmod +x /entrypoint.sh /start.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/start.sh"]