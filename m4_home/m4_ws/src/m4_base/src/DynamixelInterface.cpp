#include <stdio.h>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "utils.h"
#include "ConsoleFormatting.h"
#include "DynamixelInterface.h"


DynamixelPositionInterface::DynamixelPositionInterface()
{
	initialized = false;
}

DynamixelPositionInterface::DynamixelPositionInterface(const char* port, const int &baudrate, const std::vector<int> &ids)
{
	init(port, baudrate, ids);
}

void DynamixelPositionInterface::init(const char* port, const int &baudrate, const std::vector<int> &ids)
/*
	port: Serial port name
	baudrate: Serial port baudrate set on Dynamixel using Dynamixel Wizard
	ids: List of motor IDs to connect to (IDs set on Dynamixel using Dynamixel Wizard).
*/
{
	active_joint_ids = ids;
	joint_configs.resize(active_joint_ids.size());
	active_joint_names.resize(active_joint_ids.size());
	
	// Load configuration variables
	//---------------------------------------------------------------------------------------------------------------
	// PROJECT_ROOT_DIR set by CMakeLists.txt

	printf("[INFO] Loading Dynamixel Params...\n");

	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/motor_config.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);
	
	ADDR_GOAL_POSITION = config["ADDR_GOAL_POSITION"].as<int>();
	ADDR_TORQUE_ENABLE = config["ADDR_TORQUE_ENABLE"].as<int>();
	ADDR_MOVEMENT_SPEED = config["ADDR_MOVEMENT_SPEED"].as<int>();
	ADDR_PRESENT_POSITION = config["ADDR_PRESENT_POSITION"].as<int>();
	ADDR_PRESENT_VELOCITY = config["ADDR_PRESENT_VELOCITY"].as<int>();
	ADDR_PRESENT_CURRENT = config["ADDR_PRESENT_CURRENT"].as<int>();
	LEN_TORQUE_ENABLE = config["LEN_TORQUE_ENABLE"].as<int>();
	LEN_GOAL_POSITION = config["LEN_GOAL_POSITION"].as<int>();
	LEN_MOVEMENT_SPEED = config["LEN_MOVEMENT_SPEED"].as<int>();
	LEN_PRESENT_POSITION = config["LEN_PRESENT_POSITION"].as<int>();
	LEN_PRESENT_VELOCITY = config["LEN_PRESENT_VELOCITY"].as<int>();
	LEN_PRESENT_CURRENT = config["LEN_PRESENT_CURRENT"].as<int>();
	MULT_SPEED = config["MULT_SPEED"].as<float>();
	MULT_POSITION = config["MULT_POSITION"].as<float>();
	MULT_CURRENT = config["MULT_CURRENT"].as<float>();
	PROTOCOL_VERSION = config["PROTOCOL_VERSION"].as<float>();
	TORQUE_ENABLE = config["TORQUE_ENABLE"].as<int>();
	TORQUE_DISABLE = config["TORQUE_DISABLE"].as<int>();

	printf("[INFO] Loaded Dynamixel Params.\n");

	printf("[INFO] Loading Robot Joint Params...\n");

	configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
	config = YAML::LoadFile(configFilePath);

	std::vector<std::string> joint_names = config["joint_names"].as<std::vector<std::string>>();

	for (int i = 0; i < joint_names.size(); i++)
	{
		int jid = config["joints"][joint_names[i]]["id"].as<int>();
		// Check if the joint ID is in the list of active joints
		auto j_index = std::find(active_joint_ids.begin(), active_joint_ids.end(), jid);
        if(j_index != active_joint_ids.end())
		{
			JointConfig jconfig;
			jconfig.name = joint_names[i];
			jconfig.id = jid;
            jconfig.zero_tick = config["joints"][joint_names[i]]["zero_tick"].as<int>();
			jconfig.max_tick = config["joints"][joint_names[i]]["max_tick"].as<int>();
			jconfig.min_tick = config["joints"][joint_names[i]]["min_tick"].as<int>();
			jconfig.speed = config["joints"][joint_names[i]]["speed"].as<float>();
			joint_configs[j_index - active_joint_ids.begin()] = jconfig;
			active_joint_names[j_index - active_joint_ids.begin()] = joint_names[i];
		}
	}
	
	printf("%sInitializing dynamixel with settings:\n%s", CONSOLE_GREEN, CONSOLE_DEFAULT);
	printf("\tADDR_TORQUE_ENABLE: %d\n", ADDR_TORQUE_ENABLE);
	printf("\tADDR_GOAL_POSITION: %d\n", ADDR_GOAL_POSITION);
	printf("\tADDR_MOVEMENT_SPEED: %d\n", ADDR_MOVEMENT_SPEED);
	printf("\tADDR_PRESENT_POSITION: %d\n", ADDR_PRESENT_POSITION);
	printf("\tADDR_PRESENT_VELOCITY: %d\n", ADDR_PRESENT_VELOCITY);
	printf("\tADDR_PRESENT_CURRENT: %d\n", ADDR_PRESENT_CURRENT);
	printf("\tLEN_TORQUE_ENABLE: %d\n", LEN_TORQUE_ENABLE);
	printf("\tLEN_GOAL_POSITION: %d\n", LEN_GOAL_POSITION);
	printf("\tLEN_MOVEMENT_SPEED: %d\n", LEN_MOVEMENT_SPEED);
	printf("\tLEN_PRESENT_POSITION: %d\n", LEN_PRESENT_POSITION);
	printf("\tLEN_PRESENT_VELOCITY: %d\n", LEN_PRESENT_VELOCITY);
	printf("\tLEN_PRESENT_CURRENT: %d\n", LEN_PRESENT_CURRENT);
	printf("\tMULT_SPEED: %f\n", MULT_SPEED);
	printf("\tMULT_POSITION: %f\n", MULT_POSITION);
	printf("\tMULT_CURRENT: %f\n", MULT_CURRENT);
	printf("\tPROTOCOL_VERSION: %f\n", PROTOCOL_VERSION);
	printf("\tTORQUE_ENABLE: %d\n", TORQUE_ENABLE);
	printf("\tTORQUE_DISABLE: %d\n", TORQUE_DISABLE);
	printf("\n%sJoint configurations:\n%s", CONSOLE_GREEN, CONSOLE_DEFAULT);
	for(int i=0; i < joint_configs.size(); i++)
	{
		printf("\t%s:\n",joint_configs[i].name.c_str());
		printf("\t\tID: %d\n\t\tzero_tick: %d\n\t\tmin_tick: %d\n\t\tmax_tick: %d\n\t\tspeed: %f\n\n",
			joint_configs[i].id,
			joint_configs[i].zero_tick,
			joint_configs[i].min_tick,
			joint_configs[i].max_tick,
			joint_configs[i].speed);
	}
	
	//---------------------------------------------------------------------------------------------------------------

	portHandler = dynamixel::PortHandler::getPortHandler(port);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	groupSyncWritePosition = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
	groupSyncWriteMoveSpeed = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_MOVEMENT_SPEED, LEN_MOVEMENT_SPEED);

    // Connect to all motors and enable torque
	if(!connect(baudrate))
	{
		throw std::string("Failed to connect!\n");
        return;
	}
	
	// Set up read handler
	read_data_length = LEN_PRESENT_POSITION + LEN_PRESENT_VELOCITY + LEN_PRESENT_CURRENT;
	read_start_address =  ADDR_PRESENT_CURRENT;
	bool dxl_addparam_result = false;

	groupSyncRead = new dynamixel::GroupFastSyncRead(portHandler, packetHandler, read_start_address, read_data_length);

	// Add other motors read
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		dxl_addparam_result = groupSyncRead->addParam(active_joint_ids[i]);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed\n", active_joint_ids[i]);
			exit(1);
		}
	}

	// Set the speed of each motor
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		write_movement_speed(active_joint_ids[i], joint_configs[i].speed);
	}

	initialized = true;
	running = true;
	clock_start = std::chrono::high_resolution_clock::now();
}

void DynamixelPositionInterface::stop()
{
	disconnect();
	running = false;
}

DynamixelPositionInterface::~DynamixelPositionInterface()
{	
	if(running)
		stop();
}

int DynamixelPositionInterface::connect(const int &baudrate)
{
	// open port
	if(portHandler->openPort())
		printf("Succeeded to open the port\n");
	else
	{
        printf("Failed to open the port\n");
        return 0;
    }

    // set baudrate
	if(portHandler->setBaudRate(baudrate))
        printf("Succeeded to change the baudrate\n");
	else
	{
        printf("Failed to change the baudrate\n");
        return 0;
    }

	// enable torque and connect to servos
	uint8_t dxl_error = 0;  
	int dxl_comm_result = COMM_TX_FAIL;

	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, active_joint_ids[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			printf("\nUnable to connect to Dynamixel %d\n\n", active_joint_ids[i]);
			printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
			return 0;
		}
		else if (dxl_error != 0)
		{
			printf("\nUnable to connect to Dynamixel %d\n\n", active_joint_ids[i]);
			printf("%s\n", packetHandler->getRxPacketError(dxl_error));
			return 0;
		}
		else
		{
			printf("Dynamixel#%2d has been successfully connected \n", active_joint_ids[i]);
		}
	}

	running = true;

	return 1;
}

int DynamixelPositionInterface::disconnect()
{
	printf("\nDisconnecting from interface!\n");
	// disable torque and disconnect from servos
    uint8_t dxl_error = 0;  
    int dxl_comm_result = COMM_TX_FAIL;
    
	// Disconnect all other motors
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, active_joint_ids[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result!= COMM_SUCCESS)
        {
            printf("\nUnable to disconnect from Dynamixel %d\n\n", active_joint_ids[i]);
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return 0;
        }
        else if (dxl_error!= 0)
        {
            printf("\nUnable to disconnect from Dynamixel %d\n\n", active_joint_ids[i]);
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return 0;
        }
        else
		{
            printf("Dynamixel#%d has been successfully disconnected \n", active_joint_ids[i]);
        }	
	}

	portHandler->closePort();

	running = false;

	return 1;
}

int DynamixelPositionInterface::write_position(std::vector<int> ids, std::vector<float> positions)
{
	if(ids.size() != positions.size())
	{
		fprintf(stderr, "[set_goal_position] Size mismatch between list of IDs and Positions.\n");
        return 0;
	}
	if(IsSubset(active_joint_ids, ids) != true)
	{
		fprintf(stderr, "[set_goal_position] Some motor IDs provided were not initialized.\n");
        return 0;
	}

	// Convert positions from degrees to ticks and add zero offset
	for(int i = 0; i < positions.size(); i++)
	{
		JointConfig jconfig = joint_configs[std::find(active_joint_ids.begin(), active_joint_ids.end(), ids[i]) - active_joint_ids.begin()];
		positions[i] = positions[i] / MULT_POSITION + jconfig.zero_tick;
		// Warn if positions are out of range
		if(positions[i] < jconfig.min_tick)
		{
			printf("%s[ERROR]: [set_goal_position] Position %d out of range for Joint %s%s\n", CONSOLE_MSG_ERROR, int(positions[i]), jconfig.name.c_str(), CONSOLE_DEFAULT);
			printf("[WARN]: Moving to min position %d instead", jconfig.min_tick);
			positions[i] = jconfig.min_tick;
		}
		else if(positions[i] > jconfig.max_tick)
		{
			printf("%s[ERROR]: [set_goal_position] Position %d out of range for Joint %s%s\n", CONSOLE_MSG_ERROR, int(positions[i]), jconfig.name.c_str(), CONSOLE_DEFAULT);
			printf("[WARN]: Moving to max position %d instead\n", jconfig.max_tick);
			positions[i] = jconfig.max_tick;
		}
		
		// printf("[INFO]: Writing position %d=%f deg to joint %s\n", int(positions[i]), (positions[i] - jconfig.zero_tick)*MULT_POSITION, jconfig.name.c_str());
	}

	int dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;
	uint8_t param_goal_position[4];

	// Load goal positions to buffer
    for(int i = 0; i < ids.size(); i++)
	{
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(positions[i]));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(positions[i]));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(positions[i]));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(positions[i]));

		dxl_addparam_result = groupSyncWritePosition->addParam(ids[i], param_goal_position);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncWritePosition addparam failed\n", ids[i]);
			return 0;
		}
	}

	// Send buffer to servos
	dxl_comm_result = groupSyncWritePosition->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) 
	{
        fprintf(stderr, "[groupSyncWritePosition] txPacket write failed\n");
        return 0;
    }

	// Clear syncwrite parameter storage buffer
	groupSyncWritePosition->clearParam();
	return 1;
}

int DynamixelPositionInterface::write_movement_speed(int id, float movement_speed)
{
	printf("[INFO] Setting movement speed for joint id %d to %03f degrees per second\n", id, movement_speed);

	// Convert speed from degrees per second to rotations per minute
	movement_speed = movement_speed / MULT_SPEED;

	int dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
	bool dxl_addparam_result = false;
	uint8_t param_goal_velocity[4];

	// Load goal velocity to buffer
   	param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(movement_speed));
	param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(movement_speed));
	param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(movement_speed));
	param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(movement_speed));

	dxl_addparam_result = groupSyncWriteMoveSpeed->addParam(id, param_goal_velocity);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupSyncWriteMoveSpeed addparam failed", id);
		return 0;
	}
	

	// Send buffer to servos
	dxl_comm_result = groupSyncWriteMoveSpeed->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) 
	{
        fprintf(stderr, "[groupSyncWriteMoveSpeed] txPacket write failed");
        return 0;
    }

	// Clear syncwrite parameter storage buffer
	groupSyncWriteMoveSpeed->clearParam();

	printf("Speed set!\n");

	return 1;
}

DynamixelStatus DynamixelPositionInterface::read_status()
{
	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;
	bool dxl_getdata_result = false;

	// Perform a sync read to get status
	dxl_comm_result = groupSyncRead->txRxPacket();
	auto now = std::chrono::high_resolution_clock::now();
	double timestamp = get_duration_secs(clock_start, now);

	DynamixelStatus status;
	status.timestamp = timestamp;
	status.position.resize(active_joint_ids.size());
	status.velocity.resize(active_joint_ids.size());
	status.current.resize(active_joint_ids.size());
	
	// Check for comms success
	if (dxl_comm_result != COMM_SUCCESS)
	{
		fprintf(stderr, "[groupSyncRead] txRxPacket read failed\n\n");
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}

	// Check for servo errors
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		if(groupSyncRead->getError(active_joint_ids[i], &dxl_error))
		{
			printf("[ID:%03d] %s\n", active_joint_ids[i], packetHandler->getRxPacketError(dxl_error));
		}
	}

	// Save data from buffer
	for(int i = 0; i < active_joint_ids.size(); i++)
	{
		 dxl_getdata_result = groupSyncRead->isAvailable(active_joint_ids[i], read_start_address, read_data_length);
		 if (dxl_getdata_result != true)
		 {
			fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed\n", active_joint_ids[i]);
		 }

		JointConfig joint_config = get_joint_config(active_joint_ids[i]);

		status.position[i] = MULT_POSITION * (twos_complement(groupSyncRead->getData(active_joint_ids[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION), LEN_PRESENT_POSITION) - joint_config.zero_tick);
		status.velocity[i] = MULT_SPEED * twos_complement(groupSyncRead->getData(active_joint_ids[i], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY), LEN_PRESENT_VELOCITY);
		status.current[i] = MULT_CURRENT * short(twos_complement(groupSyncRead->getData(active_joint_ids[i], ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT), LEN_PRESENT_CURRENT));
	}
	
	status.joint_names = active_joint_names;

	return status;
}

JointConfig DynamixelPositionInterface::get_joint_config(int id)
{
	for(JointConfig joint : joint_configs)
	{
		if(joint.id == id)
			return joint;
	}


	return JointConfig();
}
