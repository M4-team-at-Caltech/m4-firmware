#ifndef M4_BASE_H
#define M4_BASE_H

#include <vector>
#include <string>
#include <thread>
#include "DynamixelInterface.h"
#include "Logger.h"

enum M4Commands
{
	CMD_STAND,
	CMD_CROUCH,
	CMD_CROUCH2,
	CMD_SIT,
	CMD_UAV,
	CMD_MIP,
	CMD_CUSTOM
};

enum M4Configurations
{
	INIT,
	STAND,
	CROUCH,
	SIT,
	UAV,
	MIP,
	CUSTOM
};

struct RobotState
{
	float timestamp;
	std::vector<float> positions;
	std::vector<float> velocities;
	std::vector<float> currents;

	void print();
};

class M4Base
{
	public:
		M4Base(std::vector<int> active_joint_ids, bool log_status=false);
		~M4Base();
		
		bool is_moving();
		bool state_available();
		RobotState get_state();
		M4Configurations get_current_configuration();

		// Transformations
		void stand();
		void crouch();
		void sit();
		void uav();
		void mip();
		void custom_joint_pos(const std::vector<int> &ids, const std::vector<float> &positions, float movement_time);

	private:
		DynamixelPositionInterface dxl;
		
		std::vector<RobotState> robot_state;
		M4Configurations current_configuration;

		std::vector<float> stand_configuration;
		std::vector<float> crouch_configuration;
		std::vector<float> crouch2_configuration;
		std::vector<float> sit_configuration;
		std::vector<float> uav_configuration;
		std::vector<float> mip_configuration;
		std::vector<std::vector<float>> transform_stand_to_crouch;
		std::vector<std::vector<float>> transform_stand_to_sit;
		std::vector<std::vector<float>> transform_stand_to_uav;
		std::vector<std::vector<float>> transform_stand_to_mip;

		std::vector<float> get_active_joint_positions(const std::vector<float>& joint_positions);

	// Logging related variables and functions
		bool log_status;
		std::vector<int> all_joint_ids;
		std::vector<int> active_joint_ids;
		std::vector<std::string> all_joint_names;
		std::vector<std::string> active_joint_names;
		std::vector<std::string> get_active_joint_names();
		std::thread log_thread;
		std::string LOG_PATH;
		int LOG_BUFFER_SIZE;

		Logger *position_logger;		// Handle for position logger
		Logger *velocity_logger;		// Handle for velocity logger
		Logger *current_logger;			// Handle for current logger
		
		bool state_updated;				// Flag to indicate if the state has been updated

		void set_log_status(bool log_status);
		void init_log();
		void update_state();
		void clear_logs();
		void log_status_loop();

	// Command queue related variables and functions
		CircularBuffer<M4Commands> command_queue;

		float CONTROL_LOOP_RATE;
		bool quit;

		void control_loop();
		std::thread control_thread;
		void active_sleep(double sleep_time);

		void execute_stand(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);
		void execute_crouch(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);
		void execute_crouch2(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);
		void execute_sit(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);
		void execute_uav(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);
		void execute_mip(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);
		void execute_custom(int& command_step, bool& command_active, std::chrono::_V2::system_clock::time_point& gait_start);

		std::vector<float> joint_position_cmd;
		std::vector<int> joint_position_cmd_ids;
		float movement_time;
};

#endif