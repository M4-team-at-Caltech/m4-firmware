FROM ros:humble-ros-core-jammy

# Basic packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    wget \
    curl \
    libpthread-stubs0-dev \
    python3-setuptools \
    python3-pip \
    tmux \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
   rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Install mavproxy
RUN pip3 install mavproxy future
# Install mavros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-mavros \
    && rm -rf /var/lib/apt/lists/*
# Install GeographicLib for mavros
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && bash ./install_geographiclib_datasets.sh

# Install MAVSDK
RUN wget https://github.com/mavlink/MAVSDK/releases/download/v3.8.1/libmavsdk-dev_3.8.1_debian12_arm64.deb
RUN dpkg -i libmavsdk-dev_3.8.1_debian12_arm64.deb
RUN pip3 install protobuf==5.29.0
RUN pip3 install mavsdk

# Install CycloneDDS RMW implementation
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set home directory - Mount this when running
WORKDIR /home/m4

# Style command prompt
RUN echo "export PS1='\\[\\e[1;35m\\]\\u@\\h:\\[\\e[1;34m\\]\\w\\[\\e[0m\\]\\$ '" >> ~/.bashrc

# Source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Enable CycloneDDS
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Clone and build px4_msgs
RUN echo "source /home/prance/px4_ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
ENTRYPOINT [ "bash" ]
