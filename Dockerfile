# This Dockerfile is for CI purposes only
# To build this image from the root repo path:
# docker build .
ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
 && rm -rf /var/lib/apt/lists/*

COPY package.xml /ros_ws/src/mavlink_sitl_gazebo/package.xml

WORKDIR /ros_ws/

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /

# Install mavlink
RUN git clone --recursive --depth=1 https://github.com/mavlink/mavlink.git && \
    cd mavlink && \
    pip3 install future && \
    python3 -m pymavlink.tools.mavgen --lang=C++11 --wire-protocol=2.0 \
        --output=/mavlink/include/mavlink/v2.0 message_definitions/v1.0/all.xml

WORKDIR /ros_ws/

COPY . /ros_ws/src/mavlink_sitl_gazebo

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    export CPLUS_INCLUDE_PATH=/mavlink/include/mavlink/v2.0 && \
    colcon build
