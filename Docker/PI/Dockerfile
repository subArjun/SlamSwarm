FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get --assume-yes install -y \
    python3-pip \
    python3-rosdep \
    ros-humble-nav2-bringup \
    python3-smbus

# Create and initialize ROS2 workspace
ENV ROS_WS=/opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

COPY ./slam_swarm ${ROS_WS}/src/slam_swarm

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

ENV ROS_DOMAIN_ID=4
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Custom entrypoint script
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]



