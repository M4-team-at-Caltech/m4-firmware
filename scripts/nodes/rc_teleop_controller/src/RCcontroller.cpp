#include "RCcontroller.h"

void RCController::read_radio_config(const std::string& config_file)
{
	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + config_path;
	YAML::Node config = YAML::LoadFile(configFilePath);

	drive_switch_config.channel_id = config["drive_mode"]["channel_id"].as<int>();
	drive_switch_config.state1_pwm = config["drive_mode"]["stand"].as<int>();
	drive_switch_config.state2_pwm = config["drive_mode"]["crouch1"].as<int>();
	drive_switch_config.state3_pwm = config["drive_mode"]["crouch2"].as<int>();

	uav_switch_config.channel_id = config["uav_mode"]["channel_id"].as<int>();
	uav_switch_config.state1_pwm = config["uav_mode"]["drive"].as<int>();
	uav_switch_config.state2_pwm = config["uav_mode"]["uav_transform"].as<int>();

	xvel_config.channel_id = config["x_velocity"]["channel_id"].as<int>();
	xvel_config.min_pwm = config["x_velocity"]["min"].as<int>();
	xvel_config.max_pwm = config["x_velocity"]["max"].as<int>();
	xvel_config.zero_pwm = config["x_velocity"]["zero"].as<int>();
	xvel_config.deadzone = config["velocity_deadzone"].as<float>();
	
	yvel_config.channel_id = config["y_velocity"]["channel_id"].as<int>();
	yvel_config.min_pwm = config["y_velocity"]["min"].as<int>();
	yvel_config.max_pwm = config["y_velocity"]["max"].as<int>();
	yvel_config.zero_pwm = config["y_velocity"]["zero"].as<int>();
	yvel_config.deadzone = config["velocity_deadzone"].as<float>();

	arm_switch_config.channel_id = config["arming"]["channel_id"].as<int>();
	arm_switch_config.state1_pwm = config["arming"]["disarm"].as<int>();
	arm_switch_config.state2_pwm = config["arming"]["ground_mode_arm"].as<int>();
	arm_switch_config.state3_pwm = config["arming"]["aerial_mode_arm"].as<int>();
}

void RCController::rc_callback(const mavros_msgs::msg::RCIn::SharedPtr rc_msg)
{
	// Drive Channel
	if (rc_msg.channels[drive_switch_config.channel_id-1] == drive_switch_config.state1_pwm) {
		drive_switch_state = DriveSwitchState::STAND;
	} else if (rc_msg.channels[drive_switch_config.channel_id-1] == drive_switch_config.state2_pwm) {
		drive_switch_state = DriveSwitchState::CROUCH1;
	} else if (rc_msg.channels[drive_switch_config.channel_id-1] == drive_switch_config.state3_pwm) {
		drive_switch_state = DriveSwitchState::CROUCH2;
	}

	// UAV Channel
	if (rc_msg.channels[uav_switch_config.channel_id-1] == uav_switch_config.state1_pwm) {
		uav_switch_state = UAVSwitchState::UAV_DISABLE;
	} else if (rc_msg.channels[uav_switch_config.channel_id-1] == uav_switch_config.state2_pwm) {
		uav_switch_state = UAVSwitchState::UAV_TRANSFORM;
	}

	// ARM Channel
	if (rc_msg.channels[arm_switch_config.channel_id-1] == arm_switch_config.state1_pwm) {
		arm_switch_state = ArmSwitchState::DISARM;
	} else if (rc_msg.channels[arm_switch_config.channel_id-1] == arm_switch_config.state2_pwm) {
		arm_switch_state = ArmSwitchState::GROUND_ARM;
	} else if (rc_msg.channels[arm_switch_config.channel_id-1] == arm_switch_config.state3_pwm) {
		arm_switch_state = ArmSwitchState::AERIAL_ARM;
	}

	// Velocity Joystick State normalized to [-1, 1]
	velocity_joystick_state.x_velocity = float((-xvel_config.zero + rc_msg.channels[xvel_config.channel_id-1])) / (xvel_config.max - xvel_config.zero);
	velocity_joystick_state.y_velocity = float(rc_msg.channels[yvel_config.channel_id-1] - yvel_config.zero) / (yvel_config.max - yvel_config.zero);

	// Apply deadzone
	if (abs(velocity_joystick_state.x_velocity) <= xvel_config.deadzone)
		velocity_joystick_state.x_velocity = 0;
	if (abs(velocity_joystick_state.y_velocity) <= yvel_config.deadzone)
		velocity_joystick_state.y_velocity = 0;

	rc_state_available = true;
}