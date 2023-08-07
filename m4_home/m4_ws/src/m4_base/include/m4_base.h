#ifndef M4_BASE_H
#define M4_BASE_H

#include <vector>
#include <string>
#include <thread>
#include "DynamixelInterface.h"
#include "Logger.h"

enum M4Commands
{
	STAND,
	CROUCH,
	SIT,
	UAV,
	MIP,
	CUSTOM
};

enum M4Configurations
{
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

		// Transformations
		void stand();
		void crouch();
		void sit();
		void uav();
		void mip();
		void custom_joint_pos(const std::vector<int> &ids, const std::vector<float> &positions);

	private:
		DynamixelPositionInterface dxl;
		
		std::vector<RobotState> robot_state;
		M4Configurations current_configuration;

		std::vector<float> stand_configuration;
		std::vector<float> crouch_configuration;
		std::vector<float> sit_configuration;
		std::vector<float> uav_configuration;
		std::vector<float> mip_configuration;
		std::vector<std::vector<float>> transform_stand_to_crouch;
		std::vector<std::vector<float>> transform_stand_to_sit;
		std::vector<std::vector<float>> transform_stand_to_uav;
		std::vector<std::vector<float>> transform_stand_to_mip;

	// Logging related variables and functions
		bool log_status;
		std::vector<int> active_joint_ids;
		std::vector<std::string> active_joint_names;
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
		RobotState get_state();
		void clear_logs();
		void log_status_loop();

	// Command queue related variables and functions
		CircularBuffer<M4Commands> command_queue;

		float CONTROL_LOOP_RATE;
		bool quit;

		void control_loop();
		std::thread control_thread;
		void active_sleep(double sleep_time);

		void execute_stand();
		void execute_crouch();
		void execute_sit();
		void execute_uav();
		void execute_mip();

		std::vector<float> joint_position_cmd;
		std::vector<int> joint_position_cmd_ids;
		void execute_custom_joint_pos();
};

#endif