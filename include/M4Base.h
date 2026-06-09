#ifndef M4_BASE_H
#define M4_BASE_H

#include <vector>
#include <string>
#include <thread>
#include "DynamixelInterface.h"
#include "Logger.h"
#include <mutex>

#ifdef SIMULATION
#include <array>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#endif

enum M4Commands
{
	CMD_STAND,
	CMD_CROUCH1,
	CMD_CROUCH2,
	CMD_SIT,
	CMD_UAV,
	CMD_MIP
};

enum M4Configurations
{
	INIT,
	STAND,
	CROUCH1,
	CROUCH2,
	SIT,
	UAV,
	MIP
};
static const std::vector<std::string> commands = {"STAND", "CROUCH1", "CROUCH2", "SIT", "UAV", "MIP"};

struct RobotState
{
	int64_t timestamp;

	// Base states
	std::vector<float> pb;				// Base position
	std::vector<float> qb;				// Base orientation quaternion (w, x, y, z)

	//Wheel (Thruster frame) poses in body frame
	std::vector<std::vector<float>> pl;		// Position of each wheel center (Thruster frame)
	std::vector<std::vector<float>> ql;		// Orientation of each wheel center (Thruster frame)

	// Joint states
	std::vector<float> qj;			// Position in degrees from ZERO_POSITION
	std::vector<float> qdotj;		// Velocity in degrees per second
	std::vector<float> ij;			// Current in amps

	// Wheel odometry
	std::vector<float> qw;			// Position in degrees
	std::vector<float> qdotw;		// Velocity in degrees per second
	std::vector<float> iw;			// Current in amps
	
	void print();
};

class M4Base
{
	public:
		M4Base();
		// Original hardware constructor:
		// M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status=false, bool autostart=true);
		#ifndef SIMULATION
		M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status=false, bool autostart=true);
		#else
		M4Base(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids, bool log_status, rclcpp::Node* ros_node);
		#endif
		~M4Base();
	
		// Status read functions
		bool state_available();
		RobotState get_state();
		M4Configurations get_current_configuration();
		void set_log_status(bool log_status);

		// Transformations
		void stand();
		void crouch1();
		void crouch2();
		void sit();
		void uav();
		void mip();

		// Wheels
		void drive(float lin_vel_cmd, float ang_vel_cmd);

	protected:
		void init(bool autostart=true); // Used to initialize M4Base

		// ── hw-bridge wrappers (M4HwBridge.cpp) ──
		// Route hardware calls to either DynamixelInterface (real) or ROS publishers (sim).
		void hw_write_position(const std::vector<int>& ids, const std::vector<float>& pos);
		void hw_write_velocity(const std::vector<int>& ids, const std::vector<float>& vel);
		DynamixelStatus hw_read_status();

		DynamixelInterface dxl;
		
		std::vector<RobotState> robot_state;
		M4Configurations current_configuration;

	// Configuration variables loaded from config/robot_config.yaml
		std::vector<float> stand_configuration;							// [Final Configuration]: STAND (Nominal driving configuration)
		std::vector<float> crouch1_configuration;						// [Final Configuration]: CROUCH1 (Lower body through saggital joint movements)
		std::vector<float> crouch2_configuration;						// [Final Configuration]: CROUCH2 (Lower body further through saggital joint movements)
		std::vector<float> sit_configuration;							// [Final Configuration]: SIT (Lower body to rest on landing gear)
		std::vector<float> mip_configuration;							// [Final Configuration]: MIP (Mobile Inverted Pendulum, a.k.a. Segway configuration)
		std::vector<float> uav_step1_configuration;						// [Intermediate Configuration]: Step 1 of transformation from STAND to UAV
		std::vector<float> uav_step2_configuration;						// [Intermediate Configuration]: Step 2 of transformation from STAND to UAV
		std::vector<float> uav_step3_configuration;						// [Intermediate Configuration]: Step 3 of transformation from STAND to UAV
		std::vector<float> uav_step4_configuration;						// [Intermediate Configuration]: Step 4 of transformation from STAND to UAV
		std::vector<std::vector<float>> transform_stand_to_crouch;		// [Configuration Sequence Buffer]: STAND to CROUCH
		std::vector<std::vector<float>> transform_stand_to_sit;			// [Configuration Sequence Buffer]: STAND to SIT
		std::vector<std::vector<float>> transform_stand_to_uav;			// [Configuration Sequence Buffer]: STAND to UAV
		std::vector<std::vector<float>> transform_stand_to_mip;			// [Configuration Sequence Buffer]: STAND to MIP
		std::vector<float> transition_wheel_vel_in;						// [Parameter]: Wheel velocity towards the body to aid in lifting chassis with morphing
		std::vector<float> transition_wheel_vel_out;					// [Parameter]: Wheel velocity away from the body to aid in lowering chassis with morphing
		float wheel_radius;												// [Parameter]: Wheel radius used for calculating differential drive velocity
		float wheel_base;												// [Parameter]: Distance between wheels used for calculating differential drive velocity

		std::vector<float> get_active_joint_positions(const std::vector<float>& joint_positions);
		std::vector<float> get_active_wheel_velocities(const std::vector<float>& wheel_velocities);

	// Logging related variables and functions
		bool log_status;						// Flag to indicate if data should be logged
		bool state_updated;						// Flag to indicate if the state has been updated

		Logger *qj_logger;						// Handle for joint position logger
		Logger *qdotj_logger;					// Handle for joint velocity logger
		Logger *ij_logger;						// Handle for joint current logger
		Logger *qw_logger;						// Handle for wheel velocity logger
		Logger *qdotw_logger;					// Handle for wheel velocity logger
		Logger *iw_logger;						// Handle for wheel current logger
		
		void update_state();
		virtual void update_base_state(RobotState& r);
		virtual void update_kinematics(RobotState& r);
		virtual void publish_state();
		virtual void init_log();
		void clear_logs();
		void log_status_step(bool flush=false);

		std::vector<int> all_joint_ids;
		std::vector<int> all_wheel_ids;
		std::vector<int> active_joint_ids;
		std::vector<int> active_wheel_ids;
		std::vector<std::string> all_joint_names;
		std::vector<std::string> all_wheel_names;
		std::vector<std::string> active_joint_names;
		std::vector<std::string> active_wheel_names;
		std::vector<std::string> get_active_joint_names();
		std::vector<std::string> get_active_wheel_names();
		std::string LOG_PATH;
		int LOG_BUFFER_SIZE;


	// Command queue related variables and functions
		CircularBuffer<M4Commands> command_queue;

		float CONTROL_LOOP_RATE;
		void control_loop();
		void control_step();
		bool autostart;
		std::thread control_thread;
		std::chrono::_V2::high_resolution_clock::time_point gait_start;
		M4Commands next_cmd;
		bool command_active = false;
		int command_step = 0;
		std::atomic<bool> quit;


		void execute_drive();
		void execute_stand();
		void execute_crouch1();
		void execute_crouch2();
		void execute_sit();
		void execute_uav();
		void execute_mip();

		std::vector<float> joint_position_cmd;
		std::vector<int> joint_position_cmd_ids;
		std::vector<float> wheel_velocity_cmd;
		std::mutex drive_cmd;
		float linear_velocity_cmd;
		float angular_velocity_cmd;
		std::vector<int> wheel_velocity_cmd_ids;

	#ifdef SIMULATION
		// ── Simulation: ROS node + Gazebo topic publishers ──
		rclcpp::Node* ros_node_{nullptr};
		std::string sim_topic_prefix_{"/Drone1"};   // loaded from robot_config.yaml simulation.topic_prefix
		std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> sim_hip_pubs_;
		std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> sim_leg_pubs_;
		std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 4> sim_wheel_pubs_;
		// Tracks last commanded state for hw_read_status() synthetic feedback
		std::vector<float> sim_joint_positions_;
		std::vector<float> sim_wheel_velocities_;
		// Velocity limits (loaded from robot_config.yaml simulation section)
		float sim_max_linear_vel_{1.0f};   // m/s
		float sim_max_angular_vel_{1.5f};  // rad/s
		float sim_max_wheel_vel_{10.0f};   // rad/s
	#endif
};

#endif