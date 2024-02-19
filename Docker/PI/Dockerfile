FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update &&  \
    apt-get --assume-yes install -y \
    python3-pip \
    python3-rosdep \
    ros-humble-nav2-bringup \
    python3-smbus \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rplidar-ros \
    ros-humble-tf-transformations \
    && sudo pip3 install transforms3d

# Create and initialize ROS2 workspace
ENV ROS_WS=/opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

COPY ./slam_swarm ${ROS_WS}/src/slam_swarm

# Build the workspace
RUN cd /opt/ros_ws && . /opt/ros/humble/setup.sh && colcon build


# Custom entrypoint script
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]



