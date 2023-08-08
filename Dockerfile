FROM ros:noetic-ros-base-focal

# Basic packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    wget\
    libpthread-stubs0-dev \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# Python Catkin tools
RUN sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -

# install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools\
    ros-$ROS_DISTRO-perception\
    ros-$ROS_DISTRO-realsense2-camera\
    ros-${ROS_DISTRO}-mavros\
    ros-${ROS_DISTRO}-mavros-extras\
    ros-${ROS_DISTRO}-mavros-msgs\
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib for mavros
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && bash ./install_geographiclib_datasets.sh

# Set home directory - Mount this when running
WORKDIR /home/m4
