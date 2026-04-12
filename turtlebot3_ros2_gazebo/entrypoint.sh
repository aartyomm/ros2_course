#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

# если будут workspace пакеты
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

exec "$@"
