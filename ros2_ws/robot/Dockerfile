FROM l4t-ros:humble-ros-base-r32.7
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
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN echo "deb http://ports.ubuntu.com/ubuntu-ports focal main universe" > /etc/apt/sources.list.d/ubuntu-20.04.list \
    && apt-get update && apt-get install --no-install-recommends -y gcc-8 g++-8 \
    && rm -rf /var/lib/apt/lists/* \
    && rm /etc/apt/sources.list.d/ubuntu-20.04.list

RUN apt-get update && apt-get install --no-install-recommends -y \
    cuda-nvcc-10-2 \
    cuda-cudart-dev-10-2 \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 100

RUN pip3 install cython nvidia-ml-py3

# Install range_libc

RUN git clone https://github.com/damanikjosh/range_libc.git /range_libc \
    && cd /range_libc/pywrapper \
    && WITH_CUDA=ON TRACE=ON python3 setup.py install

# # Install FSMT

# COPY free_space_motion_tube /tmp/free_space_motion_tube
# RUN cd /tmp/free_space_motion_tube \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install

WORKDIR /ros2_ws/src

RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git

# Install ros dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-humble-xacro \
    ros-humble-ackermann-msgs \
    ros-humble-robot-localization \
    ros-humble-joy-linux \
    ros-humble-teleop-twist-joy \
    ros-humble-interactive-marker-twist-server \
    ros-humble-twist-mux \
    ros-humble-realtime-tools \
    ros-humble-laser-proc \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-tf-transformations \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    
COPY robot/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
    
WORKDIR /ros2_ws

COPY src/common src/common
COPY src/common_gpu src/common_gpu
COPY src/robot src/robot

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

COPY src/navigation src/navigation

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install

COPY robot/cyclonedds.xml /cyclonedds.xml

RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc \
    && echo "export CYCLONEDDS_URI=file:///cyclonedds.xml" >> /root/.bashrc

COPY robot/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]