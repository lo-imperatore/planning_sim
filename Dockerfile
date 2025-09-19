# syntax=docker/dockerfile:1
FROM osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \  
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# (optional) default to CycloneDDS in the image
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN bash -lc 'echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc && \
              echo "[ -f /work/ws/install/setup.bash ] && source /work/ws/install/setup.bash" >> /etc/bash.bashrc'
WORKDIR /root/ws
