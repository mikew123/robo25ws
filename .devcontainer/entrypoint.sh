#!/bin/bash

set -e

source /opt/ros/$ROS_DISTRO/setup.bash

source ~/.bashrc

# Setup ros2 environment
cd ~/robo25ws
colcon build --symlink
. install/setup.bash

echo "Provided arguments: $@"

exec $@