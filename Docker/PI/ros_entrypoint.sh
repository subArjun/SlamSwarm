#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /opt/ros_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=4
ros2 launch tele_mapping tele_mapping_key.launch.py

exec "$@"
