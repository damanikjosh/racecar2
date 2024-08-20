ARG ROS_DISTRO=${ROS_DISTRO:-jazzy}

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

# Install CUDA
RUN curl -O https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && rm cuda-keyring_1.1-1_all.deb

RUN apt-get update && apt-get install -y \
    cuda-cudart-dev-12-6 \
    cuda-nvcc-12-6 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages cython nvidia-ml-py3

# Install range_libc
RUN git clone https://github.com/f1tenth/range_libc.git /range_libc \
    && cd /range_libc/pywrapper \
    && WITH_CUDA=ON TRACE=ON python3 setup.py install

# Install ros dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-ackermann-steering-controller \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joy-linux \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-interactive-marker-twist-server \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY simulator/requirements.txt /tmp/requirements.txt

# Install pip dependencies from
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

WORKDIR /ros2_ws

COPY src/common src/common
COPY src/common_gpu src/common_gpu
COPY src/simulator src/simulator

ENV GZ_VERSION=harmonic

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY simulator/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
