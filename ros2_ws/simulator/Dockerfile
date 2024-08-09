ARG ROS_DISTRO=jazzy

FROM osrf/ros:${ROS_DISTRO}-desktop-full
LABEL authors="Joshua J. Damanik <joshuajdmk@gmail.com>"
LABEL version="0.0.1"

ENV DEBIAN_FRONTEND=noninteractive

# Install some basic dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install cppzmq dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*


# Install libzmq
RUN git clone https://github.com/zeromq/libzmq.git /tmp/libzmq \
    && cd /tmp/libzmq \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && rm -rf /tmp/libzmq

# Install catch2
RUN git clone https://github.com/catchorg/Catch2.git -b v2.x /tmp/catch2 \
    && cd /tmp/catch2 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && rm -rf /tmp/catch2

# Install cppzmq
RUN git clone https://github.com/zeromq/cppzmq.git /tmp/cppzmq \
    && cd /tmp/cppzmq \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install \
    && rm -rf /tmp/cppzmq


# Install ros dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-ackermann-steering-controller \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

    
WORKDIR /ros2_ws

COPY src/common src/common
COPY src/simulator src/simulator

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GZ_VERSION=harmonic

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY simulator/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
