#!/bin/bash

# !!! not beeing used

echo "Sourcing startup scripts"
source /home/m4/.profile
source /home/m4/.bashrc

echo "Going to /home/m4/m4-firmware"

cd /home/m4/m4-firmware

echo "Creating two tmux sessions (interface, control)"

# Start two detached (-d) tmux sessions called interface and control
tmux new-session -d -s interface
tmux new-session -d -s control

echo "Starting docker on both of them"

# Start docker on both of them
tmux send-keys -t interface './run_docker.sh' Enter

sleep 10

tmux send-keys -t control './new_docker_terminal.sh' Enter

sleep 10

echo "Starting interfaces.launch on interface"

# start interfaces.launch
tmux send-keys -t interface 'cd m4_ws' Enter
tmux send-keys -t interface 'source devel/setup.bash' Enter
tmux send-keys -t interface 'roslaunch m4_base interfaces.launch' Enter

sleep 7

echo "Starting m4_control.launch on control"

# start m4_control.launch
tmux send-keys -t control 'cd m4_ws' Enter
tmux send-keys -t control 'source devel/setup.bash' Enter
tmux send-keys -t control 'roslaunch m4_base m4_control.launch' Enter

sleep 7

python3 led/on.py

echo "Robot Operational !"
