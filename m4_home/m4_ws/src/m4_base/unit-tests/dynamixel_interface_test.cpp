/*
 * File: dynamixel_interface_test.cpp
 * Author: Adarsh Salagame
 * Email: salagame.a@northeastern.edu
 * Github: https://github.com/adarshrs
 * 
 * Description: 
 * Unit test to send simple movement commands to a set of Dynamixels.
 * Goal is to test connectivity, movement, and error handling
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
#include <atomic>
#include <DynamixelInterface.h>

using namespace std;

atomic<bool> stopFlag(false);

void send_command_looped(DynamixelPositionInterface *dxl,vector<int> id_list,vector<float> goal_positions1, vector<float> goal_positions2)
{
	vector<float> goal_positions = goal_positions1;

	while (!stopFlag)
	{
		// Send a simple movement command to the Dynamixels
		dxl->write_position(id_list, goal_positions);

		if(goal_positions == goal_positions1)
			goal_positions = goal_positions2;
		else
			goal_positions = goal_positions;
			//goal_positions = goal_positions1;

		// 100ms - 	10 Hz
		// 50ms - 	20 Hz
		// 20ms - 	50 Hz
		// 10ms - 	100 Hz
		// 5ms - 	200 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

int main(int argc, char *argv[])
{
	// Port and baudrate to connect to dynamixel
	// Verify that the port name is correct. This can be different on your system
	// Baudrate is set using the Dynamixel Wizard on the Dynamixel Servo directly
	const char* port = string("/dev/ttyUSB0").c_str();
	int baudrate = 115200;
	
	// List of dynamixel IDs to actuate for this test
	vector<int> id_list = {2, 4};	
	vector<float> goal_positions1 = {0, 0}; 
	vector<float> goal_positions2 = {90, 90}; 
	
    // Create a DynamixelInterface object
	DynamixelPositionInterface dxl(port, baudrate, id_list);

    // Set the goal positions
	thread cmd_thread(send_command_looped, &dxl, id_list, goal_positions1, goal_positions2);

	cout << "Press q to quit" << endl;

	while(!stopFlag)
	{
		char c = getchar();
		if(c == 'q' || c == 'Q')
		{
            stopFlag = true;
        }
	}
	
	cmd_thread.join();
	cout << endl;
	return 0;
}
