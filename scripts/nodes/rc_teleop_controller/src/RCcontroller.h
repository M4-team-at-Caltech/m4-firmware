#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
// #include <mavros_msgs/msg/rc_in.hpp>
#include<px4_msgs/msg/input_rc.hpp>

#include<iostream>
#include <cmath>
// using namespace std::chrono_literals;
using std::placeholders::_1;
// std::bind(&RCController::rc_callback, this, std::placeholders::_1)
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

enum class DriveSwitchState{STAND, CROUCH1, CROUCH2};
enum class UAVSwitchState{UAV_DISABLE, UAV_TRANSFORM};
enum class ArmSwitchState{DISARM, GROUND_ARM, AERIAL_ARM};

struct VelocityJoystickState
{
	float x_velocity;
	float y_velocity;
};

struct RCstate{
	DriveSwitchState drive_switch_state;
	UAVSwitchState uav_switch_state;
	ArmSwitchState arm_switch_state;
	VelocityJoystickState velocity_joystick_state;
};


class RCController
{
	public:
		// TR: ROS1 remain
		// RCController(ros::NodeHandle& nh): rc_state_available(false)
		// {
		// 	// Initialize ROS subscribers
		// 	rc_sub_ = nh.subscribe("/mavros/rc/in", 10, &RCController::rc_callback, this);

		// 	// Read radio configuration
		// 	read_radio_config("/config/rc_channel_config.yaml");
		// }

		//TR: 2 Update_1
		RCController(rclcpp::Node* node) : rc_state_available(false)
		{
			// rc_sub_ = node->create_subscription<px4_msgs::msg::InputRc>(
			// 	"/fmu/out/input_rc",
			// 	10,
			// 	std::bind(&RCController::rc_callback, this, _1));
			rclcpp::QoS px4_qos(10);
			px4_qos.best_effort();
			px4_qos.transient_local();

			rc_sub_ = node->create_subscription<px4_msgs::msg::InputRc>(
				"/fmu/out/input_rc",
				px4_qos,
				std::bind(&RCController::rc_callback, this, std::placeholders::_1));


			read_radio_config("/config/rc_channel_config.yaml");
		}
		// Getter for RC State
		// bool get_rc_state_available() const { return rc_state_available; }
		bool get_rc_state_available() const { return rc_state_available.load(); }
		// TR: Wrong consist define
		// RCstate get_rc_state() const { rc_state_available = false; return { drive_switch_state, uav_switch_state, arm_switch_state, velocity_joystick_state }; }

		RCstate get_rc_state()
		{
			rc_state_available = false;
			return {drive_switch_state, uav_switch_state, arm_switch_state, velocity_joystick_state};
		}

	private:
		// Switch Configuration Variables
		RadioSwitchConfig drive_switch_config;
		RadioSwitchConfig uav_switch_config;
		RadioSwitchConfig arm_switch_config;

		// Joystick Configuration Variables
		RadioJoystickConfig xvel_config;
		RadioJoystickConfig yvel_config;

		void read_radio_config(const std::string& config_file);

		// Switch State Variables
		DriveSwitchState drive_switch_state;
		UAVSwitchState uav_switch_state;
		ArmSwitchState arm_switch_state;

		// Joystick State Variables
		VelocityJoystickState velocity_joystick_state;

		// rclcpp::Subscription::SharedPtr<mavros_msgs::msg::RCIn> rc_sub_;
		rclcpp::Subscription<px4_msgs::msg::InputRc>::SharedPtr rc_sub_;
		void rc_callback(const px4_msgs::msg::InputRc::SharedPtr rc_msg);

		// State updated flag
		std::atomic<bool> rc_state_available;
};
