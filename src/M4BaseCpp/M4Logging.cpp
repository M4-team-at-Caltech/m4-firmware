/**
 * @file M4Logging.cpp
 * @brief Data logging to CSV files
 * 
 * Manages buffered logging of joint positions, velocities, and currents to
 * timestamped CSV files for offline analysis.
 * 
 * Functions:
 * - publish_state()
 * - init_log()
 * - set_log_status()
 * - get_log_status()
 * - log_status_step()
 * - clear_logs()
 */

#include "M4Base.h"

void M4Base::publish_state()
{
	if(log_status)
	{
		log_status_step();
	}
}

void M4Base::init_log()
{
	printf("%s[INFO] Logging to %s\n", CONSOLE_MSG_INFO, LOG_PATH.c_str());
	qj_logger = new Logger("qj.csv", active_joint_names, LOG_PATH.c_str());
	qdotj_logger = new Logger("qdotj.csv", active_joint_names, LOG_PATH.c_str());
	ij_logger = new Logger("ij.csv", active_joint_names, LOG_PATH.c_str());

	qw_logger = new Logger("qw.csv", active_wheel_names, LOG_PATH.c_str());
	qdotw_logger = new Logger("qdotw.csv", active_wheel_names, LOG_PATH.c_str());
	iw_logger = new Logger("iw.csv", active_wheel_names, LOG_PATH.c_str());
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
		this->log_status = false;
		clear_logs();
	}
}

void M4Base::log_status_step(bool flush)
{
	if (!log_status) // Logging is disabled
		return;
    if (robot_state.empty()) // No data to log
		return;
    if (!flush && robot_state.size() < LOG_BUFFER_SIZE) // Buffer not full, no need to log
		return;

	std::vector<std::vector<float>> data_qj;
	std::vector<std::vector<float>> data_qdotj;
	std::vector<std::vector<float>> data_ij;

	std::vector<std::vector<float>> data_qw;
	std::vector<std::vector<float>> data_qdotw;
	std::vector<std::vector<float>> data_iw;

	std::vector<int64_t> timestamps;

	data_qj.reserve(robot_state.size());
	data_qdotj.reserve(robot_state.size());
	data_ij.reserve(robot_state.size());
	data_qw.reserve(robot_state.size());
	data_qdotw.reserve(robot_state.size());
	data_iw.reserve(robot_state.size());
	timestamps.reserve(robot_state.size());

	for (const auto& rs : robot_state)
	{
		timestamps.push_back(rs.timestamp);
		
		data_qj.push_back(rs.qj);
		data_qdotj.push_back(rs.qdotj);
		data_ij.push_back(rs.ij);

		data_qw.push_back(rs.qw);
		data_qdotw.push_back(rs.qdotw);
		data_iw.push_back(rs.iw);
	}

	if (qj_logger)      qj_logger->log_batch_entry(data_qj, timestamps);
	if (qdotj_logger)   qdotj_logger->log_batch_entry(data_qdotj, timestamps);
	if (ij_logger)      ij_logger->log_batch_entry(data_ij, timestamps);

	if (qw_logger)      qw_logger->log_batch_entry(data_qw, timestamps);
	if (qdotw_logger)   qdotw_logger->log_batch_entry(data_qdotw, timestamps);
	if (iw_logger)      iw_logger->log_batch_entry(data_iw, timestamps);

    robot_state.clear();
    state_updated = false;
}

void M4Base::clear_logs()
{
	delete qj_logger;
	delete qdotj_logger;
	delete ij_logger;

	delete qw_logger;
	delete qdotw_logger;
	delete iw_logger;
}