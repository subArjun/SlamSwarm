# Use the official ROS Humble base image
FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get --assume-yes install -y \
    ros-humble-navigation2 \
    build-essential \
    python3-pip \ 
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    #ros-humble-rplidar-ros \
    ros-humble-rmw-cyclonedds-cpp \
    cmake \
    git \
    python3-rosdep 

# Create and initialize ROS2 workspace
ENV ROS_WS=/opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

COPY ./tele_mapping ${ROS_WS}/src/tele_mapping

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Custom entrypoint script
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh 

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
