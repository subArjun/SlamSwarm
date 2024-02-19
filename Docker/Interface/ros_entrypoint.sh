#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=4
export QT_QPA_PLATFORM=offscreen 
/usr/bin/x11vnc -forever -usepw -create & 
rviz2

exec "$@"
