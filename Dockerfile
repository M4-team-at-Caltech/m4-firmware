FROM ros:noetic-ros-base-focal

# Basic packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    nano \
    wget\
    libpthread-stubs0-dev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# Python Catkin tools
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools\
    ros-$ROS_DISTRO-perception\
    ros-$ROS_DISTRO-realsense2-camera\
    ros-${ROS_DISTRO}-mavros\
    ros-${ROS_DISTRO}-mavros-extras\
    ros-${ROS_DISTRO}-mavros-msgs\
    ros-${ROS_DISTRO}-rosserial\
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib for mavros
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && bash ./install_geographiclib_datasets.sh

# Style command prompt
RUN echo "export PS1='\\[\\e[1;35m\\]\\u@\\h:\\[\\e[1;34m\\]\\w\\[\\e[0m\\]\\$ '" >> ~/.bashrc

# Set home directory - Mount this when running
WORKDIR /home/m4
