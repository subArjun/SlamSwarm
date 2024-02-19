FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update &&  \
    apt-get --assume-yes install -y \
    python3-pip \
    python3-rosdep \
    ros-humble-nav2-bringup \
    x11vnc xvfb \
    ros-humble-rmw-cyclonedds-cpp \
    && mkdir ~/.vnc && \
    touch ~/.vnc/passwd

RUN x11vnc -storepasswd "devopsil" ~/.vnc/passwd

# Custom entrypoint script
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]