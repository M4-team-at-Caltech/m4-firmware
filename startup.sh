#!/bin/bash

# Name of the Docker image
IMAGE_NAME="m4:ROS-Noetic"

# Name of the Docker container
CONTAINER_NAME="m4_ros_noetic"

# Path to the directory you want to mount inside the container
HOST_DIRECTORY_PATH="$(pwd)/m4_home"
CONTAINER_DIRECTORY_PATH="/home/m4"

echo "Starting Docker container ${CONTAINER_NAME}"

# 1. Start a detached tmux session and run a Docker container in it
tmux new-session -d -s docker "docker run -it --rm --privileged --name ${CONTAINER_NAME} \
             --network="host" \
             -v ${HOST_DIRECTORY_PATH}:${CONTAINER_DIRECTORY_PATH} \
             -v /etc/timezone:/etc/timezone:ro -v /etc/localtime:/etc/localtime:ro \
	     -v /dev:/dev \
	     --device-cgroup-rule "c 81:* rmw" \
	     --device-cgroup-rule "c 189:* rmw" \
             ${IMAGE_NAME} /bin/bash"

# Give Docker a moment to properly start
sleep 5

echo "Started Docker container"

# 2. Run a command inside the Docker container
# We need to find the Docker container ID first
CONTAINER_ID=$(docker ps -q --filter "name=${CONTAINER_NAME}")

echo "${CONTAINER_NAME} is running with id ${CONTAINER_ID}. Starting interfaces in tmux container name=interfaces"

CMD1 = "source m4_ws/devel/setup.bash && roslaunch m4_base interfaces.launch"
tmux new-session -d -s interfaces "docker exec ${CONTAINER_ID} \"${CMD1}\""

# Give interface time to start
sleep 5

echo "Interfaces started. Starting controller in tmux container name=controller"

# 3. Start a second tmux session and run another command inside the Docker container
CMD2 = "source m4_ws/devel/setup.bash && roslaunch m4_base m4_control.launch"
tmux new-session -d -s controller "docker exec ${CONTAINER_ID} \"${CMD2}\""

# Give controller time to start
sleep 5

echo "Controller started"