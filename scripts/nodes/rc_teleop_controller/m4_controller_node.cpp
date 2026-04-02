#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
// #include <ros/ros.h>
// #include <mavros_msgs/RCIn.h>
// #include <mavros_msgs/State.h>
// #include <mavros_msgs/OverrideRCIn.h>
// #include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "src/RCcontroller.h"
// #include "RCcontroller.h"
#include "M4Base.h"

// #define DRY_RUN
using namespace std::chrono_literals;
using std::placeholders::_1;

struct DriveModeTransitionHandler
{
	DriveSwitchState state;
	M4Configurations configuration;
	const char* message;
	void (M4Base::*action)();
};

struct UAVModeTransitionHandler
{
	UAVSwitchState state;
	M4Configurations configuration;
	const char* message;
	void (M4Base::*action)();
};

class M4ControllerNode : public rclcpp::Node
{
	// public:
		// M4ControllerNode(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids) : Node("m4_controller_node"), 
		// m4_configuration(M4Configurations::INIT)
	public:
	M4ControllerNode(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids)
		: Node("m4_controller_node"),
		m4_configuration(M4Configurations::INIT),
		m4(nullptr),
		rc_controller(this)
		{
			// rc_controller = RCController(this);

			// Initialize M4 base class
			#ifndef DRY_RUN
				m4 = new M4Base(active_joint_ids, active_wheel_ids, false);
			#endif

			// Define mode transition handlers
			drive_mode_transition_handler = {
				{ DriveSwitchState::STAND,   M4Configurations::STAND,   "Move to stand",   &M4Base::stand },
				{ DriveSwitchState::CROUCH1, M4Configurations::CROUCH1, "Move to crouch1", &M4Base::crouch1 },
				{ DriveSwitchState::CROUCH2, M4Configurations::CROUCH2, "Move to crouch2", &M4Base::crouch2 }
			};
			uav_mode_transition_handler = {
				{ UAVSwitchState::UAV_DISABLE, M4Configurations::UAV, "Move to stand", &M4Base::stand },
				{ UAVSwitchState::UAV_TRANSFORM, M4Configurations::UAV, "Move to uav", &M4Base::uav }
			};

			// Initialize ROS subscribers and publishers
			timer_ = this->create_wall_timer(50ms, std::bind(&M4ControllerNode::loop, this));
			state_subscriber_ = this->create_subscription<mavros_msgs::msg::State>(
      						"/mavros/state", 10, std::bind(&M4ControllerNode::state_callback, this, _1));
			arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
			rc_override_publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", 10);
		}
		// ~M4ControllerNode()
		// {
		// 	#ifndef DRY_RUN
		// 		m4->sit();
		// 		while(m4->get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));
		// 	#endif
		// 	delete m4;
		// 	RCLCPP_WARN("M4 Controller Node Shutting Down!");
		// }
		~M4ControllerNode()
		{
		#ifndef DRY_EXECUTION_MODE
			if (m4 != nullptr) {
				m4->sit();
				while (m4->get_current_configuration() != M4Configurations::SIT) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				delete m4;
				m4 = nullptr;
			}
		#endif
			RCLCPP_WARN(this->get_logger(), "M4 Controller Node Shutting Down!");
		}

	private:
		M4Configurations m4_configuration;
		M4Base *m4;
		// static const DriveModeTransitionHandler drive_mode_transition_handler[];
		// static const UAVModeTransitionHandler uav_mode_transition_handler[];
		std::vector<DriveModeTransitionHandler> drive_mode_transition_handler;
		std::vector<UAVModeTransitionHandler> uav_mode_transition_handler;

		RCController rc_controller;
		RCstate rc_state;

		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
		rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_publisher_;
	
		mavros_msgs::msg::State pixhawk_state;
		rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber_;
		void state_callback(const mavros_msgs::msg::State::SharedPtr msg){
			pixhawk_state = *msg;
		}

		void init_controller()
		{
			#ifndef DRY_RUN
				// Initialize to stand configuration

				m4->sit();
				std::cout<<"Sit complete!\n";
				while(m4->get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));
				std::cout<<"Sit wait complete!\n";

				m4->stand();
				while(m4->get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));
			#else
				m4_configuration = M4Configurations::STAND;
			#endif
			RCLCPP_WARN(this->get_logger(), "M4 Controller Node Initialized!");
		}

		void loop()
		{
			if(m4_configuration == M4Configurations::INIT)
				init_controller(); // Initialize controller on first run

			if(!rc_controller.get_rc_state_available())
				return; // No new RC state available. Do nothing.
			
			// Get the latest RC state
			// RCstate rc_state = rc_controller.get_rc_state();
			rc_state = rc_controller.get_rc_state();

			if(rc_state.arm_switch_state == ArmSwitchState::DISARM){
				// Stop moving and do nothing
				#ifndef DRY_RUN 
					m4->drive(0,0); 
				#endif
				return;
			}

			// Update current configuration
			#ifndef DRY_RUN
				m4_configuration = m4->get_current_configuration();
			#endif


			if(rc_state.arm_switch_state == ArmSwitchState::GROUND_ARM){
				if(rc_state.uav_switch_state == UAVSwitchState::UAV_DISABLE)
				{// Drive Mode
					// Make any required transformations
					for (const auto& handler : drive_mode_transition_handler)
					{
						if (rc_state.drive_switch_state == handler.state && m4_configuration != handler.configuration)
						{
							RCLCPP_WARN(this->get_logger(), "%s", handler.message);
							
							#ifndef DRY_RUN
								(m4->*handler.action)();
								m4_configuration = m4->get_current_configuration();
							#else
								m4_configuration = handler.configuration;
							#endif

							break;
						}
					}

					// Drive with joystick velocities
					#ifndef DRY_RUN
						m4->drive(rc_state.velocity_joystick_state.x_velocity, rc_state.velocity_joystick_state.y_velocity);
					#endif
				}
				else if(rc_state.uav_switch_state == UAVSwitchState::UAV_TRANSFORM)
				{// UAV Mode
					// Make any required transformations
					for (const auto& handler : uav_mode_transition_handler)
					{
						if (rc_state.uav_switch_state == handler.state && m4_configuration != handler.configuration)
						{
							RCLCPP_WARN(this->get_logger(), "%s", handler.message);
							
							#ifndef DRY_RUN
								m4->drive(0,0);
								(m4->*handler.action)();
								m4_configuration = m4->get_current_configuration();
							#else
								m4_configuration = handler.configuration;
							#endif

							break;
						}
					}
				}
			}
			else if(rc_state.arm_switch_state == ArmSwitchState::AERIAL_ARM){
				if(pixhawk_state.armed){
					return; // Do nothing if already armed
				}

				// If not in UAV configuration, override arming. IMPORTANT SAFETY CHECK!
				if(m4_configuration != M4Configurations::UAV)
				{
					RCLCPP_ERROR(this->get_logger(), "Not in UAV CONFIGURATION. Disarming!");
					auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
					arm_request->value = false;
					auto future = arming_client_->async_send_request(arm_request);
					// Wait for the result
					if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
						rclcpp::FutureReturnCode::SUCCESS)
					{
						auto response = future.get();
						if (response->success)
						{
							RCLCPP_INFO(this->get_logger(), "Vehicle disarmed");
						}
					}
				}
			}
		}
};

// enum M4Mode
// {
// 	MODE_DRIVE,
// 	MODE_MIP,
// 	MODE_UAV
// }current_mode;

// float xvel_state;
// float yvel_state;
// float velocity_deadzone;

// mavros_msgs::msg::State pixhawk_state;

// bool rc_state_available;
// bool pixhawk_state_available;
// bool drive_mode;
// bool teensy_ack;

// void pixhawk_state_subscriber_callback(mavros_msgs::msg::State state_msg)
// {
// 	pixhawk_state = state_msg;
// 	pixhawk_state_available = true;
// }

int main(int argc, char **argv)
{
	// rclcpp::init(argc, argv, "m4_controller_node");
	rclcpp::init(argc, argv);

	std::vector<int> active_joint_ids = {1,2,3,4,5,6,7,8};
	std::vector<int> active_wheel_ids = {9,10,11,12};

	auto node = std::make_shared<M4ControllerNode>(active_joint_ids, active_wheel_ids);

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}

