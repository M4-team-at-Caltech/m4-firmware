#include <string>
#include <vector>
#include <thread>
#include "yaml-cpp/yaml.h"
#include "DynamixelInterface.h"
#include "m4_base.h"

#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)

M4Base::M4Base(std::vector<int> active_joint_ids, bool log_status):active_joint_ids(active_joint_ids), log_status(log_status), quit(false), state_updated(false), command_queue(20)
{
	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);
	
	CONTROL_LOOP_RATE = config["control_loop_rate"].as<float>();
	LOG_PATH = config["log_path"].as<std::string>();
	
	LOG_BUFFER_SIZE = config["log_buffer_size"].as<int>();
	robot_state.reserve(LOG_BUFFER_SIZE);

	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/transformations.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);

	stand_configuration = config["stand"].as<std::vector<float>>();
	crouch_configuration = config["crouch"].as<std::vector<float>>();
	sit_configuration = config["sit"].as<std::vector<float>>();
	uav_configuration = config["uav"].as<std::vector<float>>();
	mip_configuration = config["mip"].as<std::vector<float>>();
	transform_stand_to_crouch = config["stand_to_crouch"].as<std::vector<std::vector<float>>>();
	transform_stand_to_sit = config["stand_to_sit"].as<std::vector<std::vector<float>>>();
	transform_stand_to_uav = config["stand_to_uav"].as<std::vector<std::vector<float>>>();
	transform_stand_to_mip = config["stand_to_mip"].as<std::vector<std::vector<float>>>();

	dxl.init(config["interface_port"].as<std::string>().c_str(), config["baudrate"].as<int>(), active_joint_ids);

	execute_stand();
	
	control_thread = std::thread(&M4Base::control_loop, this);
    set_log_status(log_status);
}

M4Base::~M4Base()
{
	quit = true;
	control_thread.join();
	dxl.stop();
}

void M4Base::control_loop()
{
	printf("%s[INFO] Starting control loop...\n", CONSOLE_MSG_INFO);

	auto t0 = std::chrono::system_clock::now();
	auto t1 = std::chrono::system_clock::now();
	auto gait_start = std::chrono::system_clock::now();
	double dt, sleep_for;

	M4Commands next_cmd;

    while(!quit)
    {
		update_state();

		// Execute commands
		if(!command_queue.empty())
		{
			next_cmd = command_queue.pop();
			printf("%s[WARN]: Got command %d\n", CONSOLE_MSG_WARN, next_cmd);

			switch(next_cmd)
			{
				case M4Commands::STAND:
					execute_stand();
                    break;
				case M4Commands::CROUCH:
					execute_crouch();
					break;
				case M4Commands::SIT:
					execute_sit();
					break;
				case M4Commands::UAV:
					execute_uav();
					break;
				case M4Commands::MIP:
					execute_mip();
					break;
				case M4Commands::CUSTOM:
					execute_custom_joint_pos();
					break;
			}
		}

		// Sleep until next loop time
		t1 = std::chrono::system_clock::now();
		dt = get_duration_secs(t0, t1);
		sleep_for = (1.0/CONTROL_LOOP_RATE - dt) * 1.0e3;
		if(sleep_for > 0)
			sleep_for += 1.0e3/CONTROL_LOOP_RATE;
		std::this_thread::sleep_for(std::chrono::milliseconds(int(sleep_for)));	
		t0 = t1;

    }
}

void M4Base::active_sleep(double sleep_time)
{
	auto t0 = std::chrono::system_clock::now();
	auto t1 = std::chrono::system_clock::now();
	auto sleep_start = std::chrono::system_clock::now();
	double dt, sleep_for;

	while(!quit && get_duration_secs(sleep_start, t0) <= sleep_time)
	{
		update_state();

		// Sleep until next loop time
		t1 = std::chrono::system_clock::now();
		dt = get_duration_secs(t0, t1);
		sleep_for = (1.0/CONTROL_LOOP_RATE - dt) * 1.0e3;
		if(sleep_for > 0)
			sleep_for += 1.0e3/CONTROL_LOOP_RATE;
		std::this_thread::sleep_for(std::chrono::milliseconds(int(sleep_for)));	
		t0 = t1;
	}
}

void M4Base::init_log()
{
	printf("%s[INFO] Logging to %s\n", CONSOLE_MSG_INFO, LOG_PATH.c_str());
	position_logger = new Logger("position.csv", active_joint_names, LOG_PATH.c_str());
	velocity_logger = new Logger("velocity.csv", active_joint_names, LOG_PATH.c_str());
	current_logger = new Logger("current.csv", active_joint_names, LOG_PATH.c_str());

	log_thread = std::thread(&M4Base::log_status_loop, this);
}

void M4Base::clear_logs()
{
	delete position_logger;
	delete velocity_logger;
	delete current_logger;
}

void M4Base::set_log_status(bool log_status)
{
	if(log_status && !this->log_status) // Setting logging on - creates new logs
	{
		printf("%s[INFO] Turning on logging...\n", CONSOLE_MSG_INFO);
		init_log();
		this->log_status = true;
	}

	else if(!log_status && this->log_status) // Setting logging off - frees up memory
	{
		printf("%s[INFO] Turning off logging\n", CONSOLE_MSG_INFO);
		this->log_status = log_status;
		log_thread.join();
		clear_logs();
	}
}

void M4Base::log_status_loop()
{
	printf("%s[INFO] Logging loop...\n", CONSOLE_MSG_INFO);
	int sleep_time = int((LOG_BUFFER_SIZE/CONTROL_LOOP_RATE) * 1000);
	while(log_status && !quit)
	{
		if(robot_state.size() >= LOG_BUFFER_SIZE)
		{
			std::vector<std::vector<float>> data_position;
			std::vector<std::vector<float>> data_velocity;
			std::vector<std::vector<float>> data_current;

			for(int i = 0; i < robot_state.size(); i++)
				{
				std::vector<float> q = robot_state[i].positions;
				q.insert(q.begin(), robot_state[i].timestamp);
				data_position.push_back(q);
			
				std::vector<float> qdot = robot_state[i].velocities;
				qdot.insert(qdot.begin(), robot_state[i].timestamp);
				data_velocity.push_back(qdot);
			
				std::vector<float> current = robot_state[i].currents;
				current.insert(current.begin(), robot_state[i].timestamp);
				data_current.push_back(current);
			}

			position_logger->log_batch_entry(data_position);
			velocity_logger->log_batch_entry(data_velocity);
			current_logger->log_batch_entry(data_current);
			
			robot_state.clear();
			state_updated = false;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
	}

	// Save the remaining data from buffer into file before stopping logging thread
	if(!robot_state.empty())
	{
		std::vector<std::vector<float>> data_position;
		std::vector<std::vector<float>> data_velocity;
		std::vector<std::vector<float>> data_current;

		for(int i = 0; i < robot_state.size(); i++)
		{
			std::vector<float> q = robot_state[i].positions;
			q.insert(q.begin(), robot_state[i].timestamp);
			data_position.push_back(q);
		
			std::vector<float> qdot = robot_state[i].velocities;
			qdot.insert(qdot.begin(), robot_state[i].timestamp);
			data_velocity.push_back(qdot);
		
			std::vector<float> current = robot_state[i].currents;
			current.insert(current.begin(), robot_state[i].timestamp);
			data_current.push_back(current);
		}

		position_logger->log_batch_entry(data_position);
		velocity_logger->log_batch_entry(data_velocity);
		current_logger->log_batch_entry(data_current);
		
		robot_state.clear();
		state_updated = false;
	}
}

void M4Base::update_state()
{
	DynamixelStatus status = dxl.read_status();
	RobotState r;
	r.timestamp = status.timestamp;
	r.positions = status.position;
	r.velocities = status.velocity;
	r.currents = status.current;
	robot_state.push_back(r);
	state_updated = true;
}

RobotState M4Base::get_state()
{
	state_updated = false;
	return robot_state.back();
}

void M4Base::custom_joint_pos(const std::vector<int> &ids, const std::vector<float> &positions)
{
	joint_position_cmd = positions;
	joint_position_cmd_ids = ids;
	command_queue.push(M4Commands::CUSTOM);
}

void M4Base::crouch(){
	command_queue.push(M4Commands::CROUCH);
}

void M4Base::stand(){
    command_queue.push(M4Commands::STAND);
}

void M4Base::sit(){
    command_queue.push(M4Commands::SIT);
}

void M4Base::uav(){
    command_queue.push(M4Commands::UAV);
}

void M4Base::mip(){
    command_queue.push(M4Commands::MIP);
}

void M4Base::execute_custom_joint_pos()
{
	dxl.write_position(joint_position_cmd_ids, joint_position_cmd);
	while(!is_moving()) active_sleep(0.01);
}

void M4Base::execute_stand()
{

	if(current_configuration == M4Configurations::CUSTOM)
	{
		printf("%s[ERROR] Transformation not defined from custom configuration to stand!\n", CONSOLE_MSG_ERROR);
		return;
    }

	switch(current_configuration)
	{
		case M4Configurations::STAND:
			return;
		case M4Configurations::CROUCH:
			dxl.write_position(active_joint_ids, std::vector<float>(active_joint_ids.size(), 0.0));
			break;
		case M4Configurations::SIT:
			dxl.write_position(active_joint_ids, std::vector<float>(active_joint_ids.size(), 0.0));
			break;
		case M4Configurations::UAV:
			dxl.write_position(active_joint_ids, crouch_configuration);
            while(!is_moving()) active_sleep(0.01);
			dxl.write_position(active_joint_ids, std::vector<float>(active_joint_ids.size(), 0.0));
			break;
		case M4Configurations::MIP:
			dxl.write_position(active_joint_ids, crouch_configuration);
            while(!is_moving()) active_sleep(0.01);
			dxl.write_position(active_joint_ids, std::vector<float>(active_joint_ids.size(), 0.0));
            break;
		default:
			printf("%s[ERROR] Transformation not defined!\n", CONSOLE_MSG_ERROR);
			return;
	}
	
	while(!is_moving()) active_sleep(0.01);
	current_configuration = M4Configurations::STAND;
}

void M4Base::execute_crouch()
{
	// TODO: Implement
}

void M4Base::execute_sit()
{
	// TODO: Implement
}

void M4Base::execute_uav()
{
	// TODO: Implement
}

void M4Base::execute_mip()
{
	// TODO: Implement
}

bool M4Base::state_available()
{
	return state_updated;
}

RobotState M4Base::get_state()
{
	state_updated = false;
	return robot_state.back();
}

bool M4Base::is_moving()
{
	return std::all_of(robot_state.back().velocities.begin(), robot_state.back().velocities.end(), [](float v){return v == 0.0;});
}

void RobotState::print()
{
	std::cout << "Robot state " << timestamp << std::endl;
	std::cout << "Position (deg): "; for(int i=0; i<positions.size(); i++) std::cout << positions[i] << " "; std::cout << std::endl;
	std::cout << "Velocities (deg/sec): "; for(int i=0; i<velocities.size(); i++) std::cout << velocities[i] << " "; std::cout << std::endl;
	std::cout << "Current (A): "; for(int i=0; i<currents.size(); i++) std::cout << currents[i] << " "; std::cout << std::endl;
	std::cout << std::endl;
}
