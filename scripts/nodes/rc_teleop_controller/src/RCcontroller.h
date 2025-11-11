#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/RCIn.h>

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
		RCController(ros::NodeHandle& nh): rc_state_available(false)
		{
			// Initialize ROS subscribers
			rc_sub_ = nh.subscribe("/mavros/rc/in", 10, &RCController::rc_callback, this);

			// Read radio configuration
			read_radio_config("/config/rc_channel_config.yaml");
		}

		// Getter for RC State
		bool get_rc_state_available() const { return rc_state_available; }
		RCstate get_rc_state() const { rc_state_available = false; return { drive_switch_state, uav_switch_state, arm_switch_state, velocity_joystick_state }; }

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

		rclcpp::Subscription::SharedPtr<mavros_msgs::msg::RCIn> rc_sub_;
		void rc_callback(const mavros_msgs::msg::RCIn::SharedPtr rc_msg);

		// State updated flag
		std::atomic<bool> rc_state_available;
};
