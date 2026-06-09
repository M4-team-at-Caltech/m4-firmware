/**
 * @file M4Init.cpp
 * @brief Initialization and teardown logic
 * 
 * Handles robot construction, configuration loading from YAML, hardware
 * initialization, and graceful shutdown procedures.
 * 
 * Functions:
 * - M4Base() [default constructor]
 * - M4Base() [parameterized constructor]
 * - ~M4Base()
 * - init()
 */

#include <string>
#include <vector>
#include <thread>
#include "yaml-cpp/yaml.h"
#include "DynamixelInterface.h"
#include "M4Base.h"

#ifdef SIMULATION
#include <std_msgs/msg/float64.hpp>
#endif

M4Base::M4Base() : linear_velocity_cmd(0.0), angular_velocity_cmd(0.0), log_status(false), quit(false), state_updated(false), command_queue(20), current_configuration(M4Configurations::INIT)
{
	// Do nothing, initialize later using init
}

// Original hardware constructor:
// M4Base::M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status, bool autostart): ...
#ifndef SIMULATION
M4Base::M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status, bool autostart)
	: active_joint_ids(active_joint_ids), active_wheel_ids(active_wheel_ids),
	  linear_velocity_cmd(0.0), angular_velocity_cmd(0.0), log_status(log_status),
	  quit(false), state_updated(false), autostart(autostart),
	  command_queue(20), current_configuration(M4Configurations::INIT)
{
	init(autostart);
}
#else
M4Base::M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status, rclcpp::Node* ros_node)
	: active_joint_ids(active_joint_ids), active_wheel_ids(active_wheel_ids),
	  linear_velocity_cmd(0.0), angular_velocity_cmd(0.0), log_status(log_status),
	  quit(false), state_updated(false), autostart(true),
	  command_queue(20), current_configuration(M4Configurations::INIT),
	  ros_node_(ros_node)
{
	init(true);
}
#endif

void M4Base::init(bool autostart)
{
	// Load configuration variables
	//---------------------------------------------------------------------------------------------------------------
	// PROJECT_ROOT_DIR set by CMakeLists.txt

	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);
	
	std::string interface_port = "";
	int baudrate = 0;

	try{
		CONTROL_LOOP_RATE = config["control_loop_rate"].as<float>();
		LOG_PATH = config["log_path"].as<std::string>();

		LOG_BUFFER_SIZE = config["log_buffer_size"].as<int>();
		robot_state.reserve(LOG_BUFFER_SIZE);

		interface_port = config["interface_port"].as<std::string>();
		baudrate = config["baudrate"].as<int>();

		all_joint_ids = config["all_joint_ids"].as<std::vector<int>>();
		all_joint_names = config["joint_names"].as<std::vector<std::string>>();
		all_wheel_ids = config["all_wheel_ids"].as<std::vector<int>>();
		all_wheel_names = config["wheel_names"].as<std::vector<std::string>>();
	}catch(const YAML::ParserException& e) {
	    std::cout << "Caught a YAML::ParserException: " << e.what() << std::endl;
	}

	active_joint_names = get_active_joint_names();
	active_wheel_names = get_active_wheel_names();
	float transition_vel = 0;

	try{
		stand_configuration = get_active_joint_positions(config["config_stand"].as<std::vector<float>>());
		crouch1_configuration = get_active_joint_positions(config["config_crouch1"].as<std::vector<float>>());
		crouch2_configuration = get_active_joint_positions(config["config_crouch2"].as<std::vector<float>>());
		sit_configuration = get_active_joint_positions(config["config_sit"].as<std::vector<float>>());
		uav_step1_configuration = get_active_joint_positions(config["config_uav_step1"].as<std::vector<float>>());
		uav_step2_configuration = get_active_joint_positions(config["config_uav_step2"].as<std::vector<float>>());
		mip_configuration = get_active_joint_positions(config["config_mip"].as<std::vector<float>>());
		wheel_base = config["wheel_base"].as<float>();
		wheel_radius = config["wheel_radius"].as<float>();
		transition_vel = config["config_transition_wheel_vel"].as<float>();
	}catch(const YAML::ParserException& e) {
	    std::cout << "Caught a YAML::ParserException: " << e.what() << std::endl;
	}

	transition_wheel_vel_in = get_active_wheel_velocities({-transition_vel, -transition_vel, transition_vel, transition_vel});
	transition_wheel_vel_out = get_active_wheel_velocities({transition_vel, transition_vel, -transition_vel, -transition_vel});


	printf("[INFO] Loaded M4 Params.\n");

	//----------------------------------------------------------------------------------------------------------------


	#ifdef SIMULATION
		// dxl.init(interface_port.c_str(), baudrate, active_joint_ids, active_wheel_ids);
		try {
			sim_topic_prefix_   = config["simulation"]["topic_prefix"].as<std::string>(sim_topic_prefix_);
			sim_max_linear_vel_ = config["simulation"]["max_linear_vel"].as<float>(sim_max_linear_vel_);
			sim_max_angular_vel_= config["simulation"]["max_angular_vel"].as<float>(sim_max_angular_vel_);
			sim_max_wheel_vel_  = config["simulation"]["max_wheel_vel"].as<float>(sim_max_wheel_vel_);
		} catch (...) {}
		for (int i = 0; i < 4; i++) {
			sim_hip_pubs_[i]   = ros_node_->create_publisher<std_msgs::msg::Float64>(
				sim_topic_prefix_ + "/servo_hip_"  + std::to_string(i), 10);
			sim_leg_pubs_[i]   = ros_node_->create_publisher<std_msgs::msg::Float64>(
				sim_topic_prefix_ + "/servo_leg_"  + std::to_string(i), 10);
			sim_wheel_pubs_[i] = ros_node_->create_publisher<std_msgs::msg::Float64>(
				sim_topic_prefix_ + "/vel_wheel_"  + std::to_string(i), 10);
		}
		sim_joint_positions_.assign(active_joint_ids.size(), 0.0f);
		sim_wheel_velocities_.assign(active_wheel_ids.size(), 0.0f);
		printf("[INFO] SIMULATION mode: Gazebo bridge publishers created (prefix=%s).\n",
			sim_topic_prefix_.c_str());
	#else
		dxl.init(interface_port.c_str(), baudrate, active_joint_ids, active_wheel_ids);
	#endif

	std::cout<<autostart;
	if (autostart) {
		control_thread = std::thread(&M4Base::control_loop, this);
	}
	
    if (log_status) {
        init_log();
    }
}

M4Base::~M4Base()
{
	quit = true;
	if(autostart)
	{
		printf("%s[INFO] Stopping control loop...\n", CONSOLE_MSG_INFO);
		control_thread.join();
		printf("%s[INFO] Control loop stopped.\n", CONSOLE_MSG_INFO);
	}

	#ifndef SIMULATION
		dxl.stop();
	#else
		// dxl.stop();
	#endif
}