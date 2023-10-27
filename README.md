# Software Use on M4
The `startup.sh` script will automatically run the various interfaces needed to control the robot.
Once the boot is complete the LED indicator lights up to indicate that the robot is operational and can be controlled via remote control.

# Concepts
The robot is operated by a master **Jetson Orin** (decision making) connected to **Cube Orange** (flight control), **Teensy 4.1** (drive wheel/rotor control), and **U2D2** (transform servo control) slaves. It is also equipped with additional sensors for later applications (see [user manual](docs/manual.pdf) for a list of components).

The [software architecture](docs/architecture.pdf) gets user inputs from a remote control and translates them into commands distributed via ROS. The slaves are left to operate freely towards execution of the commands, feedback is only sent upon acceptance or safe completion.

# Decomposition
The high level control code is in `m4_base` (contact Adarsh, Ioannis). There you will find the interfaces to ROS packages and Dynamixels.

The low level control code is in `low_level_controller` (contact Filip, Ioannis). There you will find the interfaces to the wheels, rotors and other hardware.

# Dev Know-How
For general questions, Filip, Adarsh, and Ioannis can help. For specific information, see the work decomposition above. Code critique is also welcome. Here are some of the main things to consider when using the robot:

* `startup.sh` is launched from /etc/rc.local which runs everytime the Nvidia Jetson Orin boots up. It launches two tmux sessions named `interface` and `control` **using sudo**. 
If the user wishes to see the information provided from the `interfaces.launch` and the `m4_control.launch` scripts which are running in the two tmux sessions, they must call `sudo tmux a -t interface` to attach to the interface session and `sudo tmux a -t control` to attach to the control session.
Sudo is necessary every time tmux is called. For example when looking for active sessions the user must specify `sudo tmux ls`.

* To exit the control script `ssh` into the onboard computer, and run `sudo tmux send-keys -t control 'C-c'`. This should kill the m4_control.launch script, move the robot to sit configuration and switch of the servos.
* To properly switch off the robot the user should also terminate the interfaces.launch script by running `sudo tmux send-keys -t interface 'C-c'`

* If you wish to re-launch the interfaces.launch script after it has been exited, without rebooting the onboard computer, you must **unplug the pixhawk from the orin and plug it in again** before launching the script again. The control script can then be launched as normal i.e. `sudo tmux send-keys -t control 'roslaunch m4_base m4_control.launch'`
