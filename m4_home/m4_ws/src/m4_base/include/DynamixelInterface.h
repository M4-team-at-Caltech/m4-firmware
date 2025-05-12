/*
 * File: DynamixelInterface.h
 * Author: Adarsh Salagame
 * Email: salagame.a@northeastern.edu
 * Github: https://github.com/adarshrs
 * 
 * Description: 
 * This is a header file that provides an interface to control the Dynamixel servos on Cobra. 
 * Position control is implemented using the Dynamixel SDK. Dynamixel servos are initially set up
 * using the Dynamixel Wizard to set the motor IDs and baud rates.
 * 
 * Dynamixel Servo Model: DYNAMIXEL XM540-W270-R
 * All addresses, data lengths, multipliers defined for this model of the Dynamixel servos.
 * 
 * Dynamixel SDK: 
 * https://github.com/ROBOTIS-GIT/DynamixelSDK
 */

#ifndef DYNAMIXEL_INTERFACE_H
#define DYNAMIXEL_INTERFACE_H

#include <stdio.h>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <chrono>
#include <atomic>

#include "dynamixel_sdk/dynamixel_sdk.h"

struct DynamixelStatus
{
	float timestamp;						// Time stamp in nanoseconds since epoch
	std::vector<std::string> names;			// Names
	std::vector<float> position;			// Position in degrees from ZERO_POSITION
	std::vector<float> velocity;			// Velocity in degrees per second
	std::vector<float> current;				// Current in amps
};

struct JointConfig
{
	std::string name;
	int id;
	int zero_tick;
	int max_tick;
	int min_tick;
	float speed; // Speed in degrees per second
};

struct WheelConfig
{
	std::string name;
	int id;
	int max_speed; // In encoder ticks
	int min_speed; // In encoder ticks
};

class DynamixelInterface
{
	public:
		DynamixelInterface();
		DynamixelInterface(const char* port, const int &baudrate, const std::vector<int> &joint_ids, const std::vector<int> &wheel_ids);
		~DynamixelInterface();

		void init(const char* port, const int &baudrate, const std::vector<int> &joint_ids, const std::vector<int> &wheel_ids);

		// Command functions
		int toggle_torque(std::vector<int> ids, bool state);
		int write_position(std::vector<int> ids, std::vector<float> positions);
		int write_movement_speed(int id, float movement_speed);
		int write_velocity(std::vector<int> ids, std::vector<float> velocities);

		// Read feedback from servos
		DynamixelStatus read_status();

		JointConfig get_joint_config(int id);
		WheelConfig get_wheel_config(int id);

		void stop();

	protected:
		// Configuration Variables
			// Control table address
			int ADDR_TORQUE_ENABLE;                 // Control table address is different in Dynamixel model
			int ADDR_GOAL_POSITION;
			int ADDR_GOAL_VELOCITY;
			int ADDR_MOVEMENT_SPEED;
			int ADDR_MOVING_STATUS;
			int ADDR_PRESENT_CURRENT;
			int ADDR_PRESENT_VELOCITY;
			int ADDR_PRESENT_POSITION;
				// Data Byte Length
			int LEN_TORQUE_ENABLE;
			int LEN_GOAL_POSITION;
			int LEN_GOAL_VELOCITY;
			int LEN_MOVEMENT_SPEED;
			int LEN_MOVING_STATUS;
			int LEN_PRESENT_POSITION;
			int LEN_PRESENT_VELOCITY;
			int LEN_PRESENT_CURRENT;
				// Units multiplier
				// Multiply to convert to, divide to convert from given units
			float MULT_SPEED;				// Multiply to convert from rotations per minute to degrees per second
			float MULT_POSITION;			// Multiply to convert from ticks to degrees
			float MULT_VELOCITY;			// Multiply to convert from encoder ticks to degrees per second
			float MULT_CURRENT;				// Multiply to convert from Dynamixel's unknown units to amps
				// Protocol version
			float PROTOCOL_VERSION;         // See which protocol version is used in the Dynamixel
				// Default setting
			int TORQUE_ENABLE;              // Value for enabling the torque
			int TORQUE_DISABLE;             // Value for disabling the torque
	
			// Flags
			bool initialized;
			bool running;

		// Dynamixel Interface handles
		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;
		dynamixel::GroupSyncRead *groupSyncRead;
		dynamixel::GroupSyncRead *groupSyncReadMoveStatus;		
		dynamixel::GroupSyncWrite *groupSyncWritePosition;
		dynamixel::GroupSyncWrite *groupSyncWriteMoveSpeed;
		dynamixel::GroupSyncWrite *groupSyncWriteVelocity;

		// Status read address and data length
		int read_start_address;
		int read_data_length;

		// Active Joints
		std::vector<int> active_joint_ids;
		std::vector<std::string> active_joint_names;
		std::vector<JointConfig> joint_configs;

		// Active Wheels
		std::vector<int> active_wheel_ids;
		std::vector<std::string> active_wheel_names;
		std::vector<WheelConfig> wheel_configs;

		std::vector<std::string> servo_names;

		// Current status
		DynamixelStatus dxl_status;

		std::chrono::time_point<std::chrono::high_resolution_clock> clock_start;
		
		int connect(const int &baudrate);
		int disconnect();
};

#endif