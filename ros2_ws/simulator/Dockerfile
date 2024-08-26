FROM ros:humble-ros-base
LABEL authors="Joshua J. Damanik <joshuajdmk@gmail.com>"
LABEL version="0.0.1"


ENV DEBIAN_FRONTEND=noninteractive

RUN if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then \
      sed -i 's|http://ports.ubuntu.com/ubuntu-ports|http://ftp.kaist.ac.kr/ubuntu-ports|g' /etc/apt/sources.list.d/ubuntu.sources; \
    else \
      sed -i 's|http://ports.ubuntu.com/ubuntu-ports|http://ftp.kaist.ac.kr/ubuntu-ports|g' /etc/apt/sources.list; \
    fi

# Install some basic dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    curl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# nvidia-l4t-core is a dependency for the rest
# of the packages, and is designed to be installed directly
# on the target device. This because it parses /proc/device-tree
# in the deb's .preinst script. Looks like we can bypass it though:

RUN apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
    && echo "deb https://repo.download.nvidia.com/jetson/common r35.5 main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && echo "deb https://repo.download.nvidia.com/jetson/t194 r35.5 main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list \
    && mkdir -p /opt/nvidia/l4t-packages/ \
    && touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall \
    && apt-get update \
    && apt install -y nvidia-l4t-cuda cuda-nvcc-11-4 \
    && rm -rf /var/lib/apt/lists/*
    

RUN pip3 install cython nvidia-ml-py3

# Install range_libc
RUN git clone https://github.com/f1tenth/range_libc.git /range_libc \
    && cd /range_libc/pywrapper \
    && WITH_CUDA=ON TRACE=ON python3 setup.py install
    

# Install pip dependencies from
RUN pip3 install setuptools==65.5.0 pip==21


COPY simulator/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY simulator/f1tenth_gym /tmp/f1tenth_gym
RUN pip3 install /tmp/f1tenth_gym --no-cache-dir \
    && rm -rf /tmp/f1tenth_gym

# Install ros dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-tf-transformations \
    ros-humble-realtime-tools \
    ros-humble-ackermann-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-joy-linux \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src/common src/common
COPY src/common_gpu src/common_gpu
COPY src/simulator src/simulator

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY simulator/cyclonedds.xml /cyclonedds.xml

COPY simulator/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
