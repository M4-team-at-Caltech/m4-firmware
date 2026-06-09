#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include<px4_msgs/msg/input_rc.hpp>

#include<iostream>
#include <cmath>
using std::placeholders::_1;

struct RadioSwitchConfig
{
	int channel_id;
	int state1_pwm;
	int state2_pwm;
	int state3_pwm;
};

struct RadioJoystickConfig
{
	int channel_id;
	int min_pwm;
	int zero_pwm;
	int max_pwm;
	float deadzone;
};

enum class DriveSwitchState { STAND, CROUCH1, CROUCH2 };
enum class UAVSwitchState   { UAV_DISABLE, UAV_TRANSFORM };
enum class ArmSwitchState   { DISARM, GROUND_ARM, AERIAL_ARM };
enum class AutoSwitchState  { MANUAL, AUTO };  // AUTO mode switch

struct VelocityJoystickState
{
	float x_velocity;
	float y_velocity;
};

struct RCstate {
	DriveSwitchState      drive_switch_state;
	UAVSwitchState        uav_switch_state;
	ArmSwitchState        arm_switch_state;
	AutoSwitchState       auto_switch_state;       // NEW: manual vs auto mode
	VelocityJoystickState velocity_joystick_state;
};


class RCController
{
	public:
		//TR: 2 Update_1
		RCController(rclcpp::Node* node) : rc_state_available(false)
		{
#ifndef SIMULATION
			rclcpp::QoS px4_qos(10);
			px4_qos.best_effort();
			px4_qos.transient_local();

			rc_sub_ = node->create_subscription<px4_msgs::msg::InputRc>(
				"/fmu/out/input_rc",
				px4_qos,
				std::bind(&RCController::rc_callback, this, std::placeholders::_1));

			read_radio_config("/config/rc_channel_config.yaml");
#endif
		}

		bool get_rc_state_available() const {
#ifdef SIMULATION
			return true;
#else
			return rc_state_available.load();
#endif
		}

		RCstate get_rc_state()
		{
#ifdef SIMULATION
			return { DriveSwitchState::STAND,
			         UAVSwitchState::UAV_DISABLE,
			         ArmSwitchState::GROUND_ARM,
			         AutoSwitchState::AUTO,
			         {0.0f, 0.0f} };
#else
			rc_state_available = false;
			return { drive_switch_state, uav_switch_state, arm_switch_state,
			         auto_switch_state, velocity_joystick_state };
#endif
		}

	private:
		// Switch Configuration Variables
		RadioSwitchConfig drive_switch_config;
		RadioSwitchConfig uav_switch_config;
		RadioSwitchConfig arm_switch_config;
		RadioSwitchConfig auto_switch_config;  // NEW: 2-state switch (state1=manual, state2=auto)

		// Joystick Configuration Variables
		RadioJoystickConfig xvel_config;
		RadioJoystickConfig yvel_config;

		void read_radio_config(const std::string& config_file);

		// Switch State Variables
		DriveSwitchState      drive_switch_state;
		UAVSwitchState        uav_switch_state;
		ArmSwitchState        arm_switch_state;
		AutoSwitchState       auto_switch_state{AutoSwitchState::MANUAL};  // default MANUAL

		// Joystick State Variables
		VelocityJoystickState velocity_joystick_state;

		rclcpp::Subscription<px4_msgs::msg::InputRc>::SharedPtr rc_sub_;
		void rc_callback(const px4_msgs::msg::InputRc::SharedPtr rc_msg);

		// State updated flag
		std::atomic<bool> rc_state_available;
};
