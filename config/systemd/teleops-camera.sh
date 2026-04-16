#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
cd /home/ttsalo/teleops/ros2_ws
source install/setup.bash
exec ros2 launch teleops camera.launch.py
