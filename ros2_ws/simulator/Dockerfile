FROM l4t-ros:humble-ros-base-r32.7
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
    cuda-samples-10-2 \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 100 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 100    

RUN pip3 install cython nvidia-ml-py3

# Install range_libc
RUN git clone https://github.com/f1tenth/range_libc.git /range_libc \
    && cd /range_libc/pywrapper \
    && WITH_CUDA=ON TRACE=ON python3 setup.py install


RUN pip3 install scipy==1.11.0

RUN git clone --recurse-submodules https://github.com/osqp/cuosqp.git /cuosqp \
    && cd /cuosqp \
    && CUDA_PATH=/usr/local/cuda-10.2 python3 setup.py install

# Install pip dependencies from
RUN pip3 install setuptools==65.5.0 pip==21


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
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

COPY simulator/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt
# Install FSMT

# COPY free_space_motion_tube /free_space_motion_tube
# RUN cd /free_space_motion_tube \
#     && mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install


WORKDIR /ros2_ws

COPY src/common src/common
COPY src/common_gpu src/common_gpu
COPY src/simulator src/simulator

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install 

COPY src/navigation src/navigation

RUN . /opt/ros/humble/setup.sh \
    && colcon build --symlink-install
    
COPY simulator/cyclonedds.xml /cyclonedds.xml

RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc \
    && echo "export CYCLONEDDS_URI=file:///cyclonedds.xml" >> /root/.bashrc


COPY simulator/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
