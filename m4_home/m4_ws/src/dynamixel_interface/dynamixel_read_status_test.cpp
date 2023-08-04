/*
 * File: dynamixel_read_status_test.cpp
 * Author: Adarsh Salagame
 * Email: salagame.a@northeastern.edu
 * Github: https://github.com/adarshrs
 * 
 * Description: 
 * Unit test to log data from dynamixel. Oscilates some motors and stores the data
 * in a log file.
 * 
 * Dynamixel Servo Model: DYNAMIXEL XM540-W270-R
 * 
 * Dynamixel SDK: 
 * https://github.com/ROBOTIS-GIT/DynamixelSDK
 */

#include <stdio.h>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <atomic>
#include <DynamixelInterface.h>

using namespace std;

atomic<bool> stopFlag(false);

void read_status(DynamixelPositionInterface *dynamixel_interface)
{	
	ofstream posLog("./Logs/position.csv", ios::out);
	ofstream velLog("./Logs/velocity.csv", ios::out);
	ofstream curLog("./Logs/current.csv", ios::out);

	while(!stopFlag)
	{
		if(dynamixel_interface->status_available())
		{
			DynamixelStatus robot_state = dynamixel_interface->get_status();

			posLog << robot_state.timestamp;
			velLog << robot_state.timestamp;
			curLog << robot_state.timestamp;
			for(int i=0; i<robot_state.position.size(); i++)
			{
				posLog << ',' << robot_state.position[i];
				velLog << ',' << robot_state.velocity[i];
				curLog << ',' << robot_state.current[i];
			}
			
			posLog << endl;
			velLog << endl;
			curLog << endl;
		}
        
		// 100ms - 	10 Hz
		// 50ms - 	20 Hz
		// 20ms - 	50 Hz
		// 10ms - 	100 Hz
		// 5ms - 	200 Hz
		this_thread::sleep_for(chrono::milliseconds(5));
	}

    posLog.close();
	velLog.close();
	curLog.close();

	cout << "Stopping read_status thread" << endl;
}

void send_command_looped(DynamixelPositionInterface *dxl,vector<int> id_list,vector<float> goal_positions)
{
	while (!stopFlag)
	{
		// Send a simple movement command to the Dynamixels
		dxl->write_position(id_list, goal_positions);
		for(int i=0; i<goal_positions.size(); i++)
			goal_positions[i] *= -1;

		// Wait for 2 seconds for movement to complete
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char *argv[])
{
	// Port and baudrate to connect to dynamixel
	// Verify that the port name is correct. This can be different on your system
	// Baudrate is set using the Dynamixel Wizard on the Dynamixel Servo directly
	const char* port = string("/dev/ttyUSB0").c_str();
	int baudrate = 4000000;
	
	// List of dynamixel IDs to actuate for this test
	vector<int> id_list = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};	
	vector<int> move_list = {1, 2};
	vector<float> goal_positions = {10, -10}; // Oscilate between +/- 20 degrees
	
    // Create a DynamixelInterface object
	DynamixelPositionInterface dxl(port, baudrate, id_list);
	dxl.enable_debug_msgs();

	// Start the read_status thread
	thread status_thread(read_status, &dxl);

	// Set movement speed in degrees per second
	dxl.write_movement_speed(60);

	// Set the goal positions
	thread cmd_thread(send_command_looped, &dxl, move_list, goal_positions);

	cout << "Press q to quit" << endl;

	while(!stopFlag)
	{
		char c = getchar();
		if(c == 'q' || c == 'Q')
		{
            stopFlag = true;
        }
	}
	
	status_thread.join();
	cmd_thread.join();

	cout << endl;
	return 0;
}