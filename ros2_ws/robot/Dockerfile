ARG ROS_DISTRO=jazzy

FROM ros:jazzy-ros-base AS ros-base
LABEL authors="Joshua J. Damanik <joshuajdmk@gmail.com>"
LABEL version="0.0.1"

# Conditional sed command based on the existence of /etc/apt/sources.list.d/ubuntu.sources
RUN if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
      sed -i 's|http://ports.ubuntu.com/ubuntu-ports|http://ftp.kaist.ac.kr/ubuntu-ports|g' /etc/apt/sources.list.d/ubuntu.sources; \
    else \
      sed -i 's|http://ports.ubuntu.com/ubuntu-ports|http://ftp.kaist.ac.kr/ubuntu-ports|g' /etc/apt/sources.list; \
    fi

ENV DEBIAN_FRONTEND=noninteractive

# Install some basic dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*


# FROM ros-base AS cuda-builder

# # Install CUDA
# RUN curl -O https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/arm64/cuda-keyring_1.1-1_all.deb \
#     && dpkg -i cuda-keyring_1.1-1_all.deb \
#     && rm cuda-keyring_1.1-1_all.deb

# RUN apt-get update && apt-get install -y \
#     cuda-cudart-dev-12-6 \
#     cuda-nvcc-12-6 \
#     && rm -rf /var/lib/apt/lists/*

# RUN pip3 install cython nvidia-ml-py3

# # Install range_libc
# RUN git clone https://github.com/f1tenth/range_libc.git /range_libc \
#     && cd /range_libc/pywrapper \
#     && WITH_CUDA=ON TRACE=ON python3 setup.py build_ext

# WORKDIR /ros2_ws/src

# COPY src/common_gpu src/common_gpu

# RUN . /opt/ros/jazzy/setup.sh \
#     && colcon build --symlink-install \
#     && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc


FROM ros-base AS robot

# COPY --from=cuda-builder /ros2_ws /ros2_ws
# COPY --from=cuda-builder /range_libc/pywrapper/build/lib.linux-aarch64-3.12 /usr/local/lib/python3.12/dist-packages

WORKDIR /ros2_ws/src

# Clone micro ROS
# RUN git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs
# RUN git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git

# Install ros dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-jazzy-xacro \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-robot-localization \
    ros-jazzy-joy-linux \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-interactive-marker-twist-server \
    ros-jazzy-twist-mux \
    ros-jazzy-realtime-tools \
    ros-jazzy-laser-proc \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-serial-driver \
    ros-jazzy-asio-cmake-module \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    
WORKDIR /ros2_ws

COPY src/common src/common
COPY src/robot src/robot

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY robot/cyclonedds.xml /cyclonedds.xml

COPY robot/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
