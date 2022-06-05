# This Dockerfile is for CI purposes only
# To build this image from the root repo path:
# docker build .
ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

COPY package.xml /ros_ws/src/mavlink_sitl_gazebo/package.xml

WORKDIR /ros_ws/

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
 && rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"
COPY . /ros_ws/src/mavlink_sitl_gazebo

RUN colcon build
