ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base
LABEL authors="Joshua J. Damanik <joshuajdmk@gmail.com>"
LABEL version="0.0.1"

RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|http://ftp.kaist.ac.kr/ubuntu-ports|g' /etc/apt/sources.list.d/ubuntu.sources

ENV DEBIAN_FRONTEND=noninteractive

# Install some basic dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

# Clone micro ROS
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_msgs
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro-ROS-Agent
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git

# Install ros dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-laser-proc \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

    
WORKDIR /ros2_ws

COPY src/common src/common
COPY src/robot src/robot

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY robot/disable_fastdds_shm.xml /tmp/


COPY robot/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
