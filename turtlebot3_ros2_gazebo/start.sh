#!/bin/bash

set -e

# виртуальный экран
Xvfb :1 -screen 0 1280x720x24 &

# оконный менеджер
fluxbox &

# VNC сервер
x11vnc -display :1 -nopw -forever -shared -bg

# noVNC web клиент
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &

source /opt/ros/jazzy/setup.bash

echo "================================="
echo "noVNC: http://localhost:6080"
echo "================================="

# ВАЖНО: запускаем Gazebo + TurtleBot3 world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

tail -f /dev/null
