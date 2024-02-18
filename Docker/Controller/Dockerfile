# Use the official ROS Humble base image
FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get --assume-yes install -y \
    ros-humble-navigation2 \
    python3-pip \ 
    ros-humble-nav2-bringup \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    python3-rosdep 

# Create and initialize ROS2 workspace
ENV ROS_WS=/opt/ros_ws
RUN mkdir -p ${ROS_WS}/src

COPY ./tele_mapping ${ROS_WS}/src/tele_mapping
#RUN rm -rf /usr/src/gmock && rm -rf /usr/src/gtest
# Build the workspace
RUN cd /opt/ros_ws && . /opt/ros/humble/setup.sh && colcon build
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Custom entrypoint script
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh 

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
