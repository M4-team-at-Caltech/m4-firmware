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

void send_command_looped(DynamixelPositionInterface *dxl,vector<int> id_list,vector<float> goal_positions)
{
	bool latch_engage=false;

	while (!stopFlag)
	{
		// Send a simple movement command to the Dynamixels
		dxl->write_position(id_list, goal_positions);

		for(int i=0; i<goal_positions.size(); i++)
		{
			goal_positions[i] *= -1;
		}
		
		latch_engage = !latch_engage;

		// 100ms - 	10 Hz
		// 50ms - 	20 Hz
		// 20ms - 	50 Hz
		// 10ms - 	100 Hz
		// 5ms - 	200 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
	vector<int> id_list = {1, 2};	
	vector<float> goal_positions = {10, -10}; // Oscilate between +/- 20 degrees
	
    // Create a DynamixelInterface object
	DynamixelPositionInterface dxl(port, baudrate, id_list);

	// Set movement speed in degrees per second
	dxl.write_movement_speed(60.0);

    // Set the goal positions
	thread cmd_thread(send_command_looped, &dxl, id_list, goal_positions);

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
