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
	float timestamp;				// Time stamp in nanoseconds since epoch
	std::vector<std::string> joint_names;// Joint names
	std::vector<float> position;	// Position in degrees from ZERO_POSITION
	std::vector<float> velocity;	// Velocity in degrees per second
	std::vector<float> current;		// Current in amps
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

class DynamixelPositionInterface
{
	public:
		DynamixelPositionInterface();
		DynamixelPositionInterface(const char* port, const int &baudrate, const std::vector<int> &ids);
		~DynamixelPositionInterface();

		void init(const char* port, const int &baudrate, const std::vector<int> &ids);

		// Read feedback from servos
		DynamixelStatus read_status();

		// Command functions
		int write_position(std::vector<int> ids, std::vector<float> positions);
		int write_movement_speed(int id, float movement_speed);

		void stop();
		
	private:
		// Configuration Variables
			// Control table address
		int ADDR_TORQUE_ENABLE;                 // Control table address is different in Dynamixel model
		int ADDR_GOAL_POSITION;
		int ADDR_MOVEMENT_SPEED;
		int ADDR_PRESENT_CURRENT;
		int ADDR_PRESENT_VELOCITY;
		int ADDR_PRESENT_POSITION;
			// Data Byte Length
		int LEN_TORQUE_ENABLE;
		int LEN_GOAL_POSITION;
		int LEN_MOVEMENT_SPEED;
		int LEN_PRESENT_POSITION;
		int LEN_PRESENT_VELOCITY;
		int LEN_PRESENT_CURRENT;
			// Units multiplier
			// Multiply to convert to, divide to convert from given units
		float MULT_SPEED;				// Multiply to convert from rotations per minute to degrees per second
		float MULT_POSITION;			// Multiply to convert from ticks to degrees
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
		dynamixel::GroupSyncWrite *groupSyncWritePosition;
		dynamixel::GroupSyncWrite *groupSyncWriteMoveSpeed;
		dynamixel::GroupSyncRead *groupSyncRead;
		
		// Motor IDs being managed
		std::vector<int> active_joint_ids;
		std::vector<std::string> active_joint_names;
		std::vector<JointConfig> joint_configs;
		
		// Status read address and data length
		int read_start_address;
		int read_data_length;

		// Current status
		DynamixelStatus dxl_status;

		std::chrono::time_point<std::chrono::high_resolution_clock> clock_start;
		
		int connect(const int &baudrate);
		int disconnect();

};

#endif