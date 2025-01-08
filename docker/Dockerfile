FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt upgrade -y

# Essential tools
RUN apt install -y \
    build-essential \
    cmake \
    git \
    gedit \
    unzip \
    curl \
    vim \
    wget \
    software-properties-common \
    libboost-all-dev \
    libeigen3-dev \
    python3-catkin-tools

# ROS
RUN apt install -y ros-noetic-navigation \
                ros-noetic-robot-localization \
                ros-noetic-robot-state-publisher

# catkin workspace
ARG CATKIN_WS_DIR=/root/catkin_ws
RUN . /opt/ros/noetic/setup.sh && mkdir -p ${CATKIN_WS_DIR}/src \
    && cd ${CATKIN_WS_DIR} \
    && catkin build \
    && echo "source ${CATKIN_WS_DIR}/devel/setup.bash" >> ~/.bashrc

# gtsam
RUN apt update
RUN add-apt-repository ppa:borglab/gtsam-release-4.0
RUN apt update
RUN apt install -y libgtsam-dev libgtsam-unstable-dev
RUN rm -rf /var/lib/apt/lists/*

# livox_ros_driver
RUN git clone https://github.com/Livox-SDK/livox_ros_driver ${CATKIN_WS_DIR}/src/livox_ros_driver
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git ${CATKIN_WS_DIR}/src/Livox-SDK2
RUN cd ${CATKIN_WS_DIR}/src/Livox-SDK2/ && mkdir build && cd build && cmake .. && make -j && make install

# ceres-solver
RUN git clone https://ceres-solver.googlesource.com/ceres-solver -b 2.0.0 ${CATKIN_WS_DIR}/src/ceres-solver
RUN apt update
RUN apt install -y libgoogle-glog-dev libgflags-dev
RUN apt install -y libatlas-base-dev
RUN cd ${CATKIN_WS_DIR}/src/ceres-solver/ && mkdir build && cd build && cmake .. && make -j3 && make install

# SC-LIO-SAM
RUN git clone https://github.com/dongjae0107/SC-LIO-SAM.git ${CATKIN_WS_DIR}/src/SC-LIO-SAM
RUN cd ${CATKIN_WS_DIR} && catkin build lio_sam

# FAST-LIO-SLAM
RUN git clone https://github.com/dongjae0107/FAST_LIO_SLAM.git ${CATKIN_WS_DIR}/src/FAST_LIO_SLAM
# RUN cd ${CATKIN_WS_DIR}/src/FAST_LIO_SLAM && git submodule update --init
RUN cd ${CATKIN_WS_DIR}/src/FAST_LIO_SLAM
RUN cd ${CATKIN_WS_DIR} && catkin build livox_ros_driver fast_lio aloam_velodyne

# LT-mapper
RUN git clone https://github.com/dongjae0107/lt-mapper ${CATKIN_WS_DIR}/src/lt-mapper
RUN cd ${CATKIN_WS_DIR} && catkin build ltslam removert

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


ENTRYPOINT [ "/bin/bash" ]