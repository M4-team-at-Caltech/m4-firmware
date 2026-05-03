#include "RCcontroller.h"
using namespace std;

void RCController::read_radio_config(const std::string& config_file)
{
	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + config_file;
	YAML::Node config = YAML::LoadFile(configFilePath);

	drive_switch_config.channel_id = config["drive_mode"]["channel_id"].as<int>();
	drive_switch_config.state1_pwm = config["drive_mode"]["stand"].as<int>();
	drive_switch_config.state2_pwm = config["drive_mode"]["crouch1"].as<int>();
	drive_switch_config.state3_pwm = config["drive_mode"]["crouch2"].as<int>();

	uav_switch_config.channel_id = config["uav_mode"]["channel_id"].as<int>();
	uav_switch_config.state1_pwm = config["uav_mode"]["drive"].as<int>();
	uav_switch_config.state2_pwm = config["uav_mode"]["uav_transform"].as<int>();

	xvel_config.channel_id = config["x_velocity"]["channel_id"].as<int>();
	xvel_config.min_pwm    = config["x_velocity"]["min"].as<int>();
	xvel_config.max_pwm    = config["x_velocity"]["max"].as<int>();
	xvel_config.zero_pwm   = config["x_velocity"]["zero"].as<int>();
	xvel_config.deadzone   = config["velocity_deadzone"].as<float>();

	yvel_config.channel_id = config["y_velocity"]["channel_id"].as<int>();
	yvel_config.min_pwm    = config["y_velocity"]["min"].as<int>();
	yvel_config.max_pwm    = config["y_velocity"]["max"].as<int>();
	yvel_config.zero_pwm   = config["y_velocity"]["zero"].as<int>();
	yvel_config.deadzone   = config["velocity_deadzone"].as<float>();

	arm_switch_config.channel_id = config["arming"]["channel_id"].as<int>();
	arm_switch_config.state1_pwm = config["arming"]["disarm"].as<int>();
	arm_switch_config.state2_pwm = config["arming"]["ground_mode_arm"].as<int>();
	arm_switch_config.state3_pwm = config["arming"]["aerial_mode_arm"].as<int>();

	// AUTO mode switch (2-state: manual / auto)
	auto_switch_config.channel_id = config["auto_mode"]["channel_id"].as<int>();
	auto_switch_config.state1_pwm = config["auto_mode"]["manual_pwm"].as<int>();
	auto_switch_config.state2_pwm = config["auto_mode"]["auto_pwm"].as<int>();
	auto_switch_config.state3_pwm = 0; // unused
}

void RCController::rc_callback(const px4_msgs::msg::InputRc::SharedPtr rc_msg)
{
	// Drive Channel
	if (rc_msg->values[drive_switch_config.channel_id-1] == drive_switch_config.state1_pwm) {
		drive_switch_state = DriveSwitchState::STAND;
	} else if (rc_msg->values[drive_switch_config.channel_id-1] == drive_switch_config.state2_pwm) {
		drive_switch_state = DriveSwitchState::CROUCH1;
	} else if (rc_msg->values[drive_switch_config.channel_id-1] == drive_switch_config.state3_pwm) {
		drive_switch_state = DriveSwitchState::CROUCH2;
	}

	// UAV Channel
	if (rc_msg->values[uav_switch_config.channel_id-1] == uav_switch_config.state1_pwm) {
		uav_switch_state = UAVSwitchState::UAV_DISABLE;
	} else if (rc_msg->values[uav_switch_config.channel_id-1] == uav_switch_config.state2_pwm) {
		uav_switch_state = UAVSwitchState::UAV_TRANSFORM;
	}

	// ARM Channel
	if (rc_msg->values[arm_switch_config.channel_id-1] == arm_switch_config.state1_pwm) {
		arm_switch_state = ArmSwitchState::DISARM;
	} else if (rc_msg->values[arm_switch_config.channel_id-1] == arm_switch_config.state2_pwm) {
		arm_switch_state = ArmSwitchState::GROUND_ARM;
	} else if (rc_msg->values[arm_switch_config.channel_id-1] == arm_switch_config.state3_pwm) {
		arm_switch_state = ArmSwitchState::AERIAL_ARM;
	}

	// AUTO mode switch (2-state)
	if (rc_msg->values[auto_switch_config.channel_id-1] == auto_switch_config.state1_pwm) {
		auto_switch_state = AutoSwitchState::MANUAL;
	} else if (rc_msg->values[auto_switch_config.channel_id-1] == auto_switch_config.state2_pwm) {
		auto_switch_state = AutoSwitchState::AUTO;
	}

	// Velocity Joystick State normalized to [-1, 1]
	velocity_joystick_state.x_velocity = -1.0*float((-xvel_config.zero_pwm + rc_msg->values[xvel_config.channel_id-1])) / (xvel_config.max_pwm - xvel_config.zero_pwm);
	velocity_joystick_state.y_velocity = float(rc_msg->values[yvel_config.channel_id-1] - yvel_config.zero_pwm) / (yvel_config.max_pwm - yvel_config.zero_pwm);

	// Apply deadzone
	if (std::abs(velocity_joystick_state.x_velocity) <= xvel_config.deadzone)
		velocity_joystick_state.x_velocity = 0;
	if (std::abs(velocity_joystick_state.y_velocity) <= yvel_config.deadzone)
		velocity_joystick_state.y_velocity = 0;

	rc_state_available = true;
}
