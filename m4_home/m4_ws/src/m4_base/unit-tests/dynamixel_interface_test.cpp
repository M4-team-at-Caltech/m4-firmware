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

void send_command_looped(DynamixelPositionInterface *dxl,vector<int> id_list)
{

	while (!stopFlag)
	{	
		// FL
		vector<float> goal_positions1 = {0, 90, 0, 90}; 
		dxl->write_position(id_list, goal_positions1);
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));
		vector<float> goal_positions2 = {0, 0, 0, 0}; 
		dxl->write_position(id_list, goal_positions2);
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));		
    }
}

int main(int argc, char *argv[])
{
	// Port and baudrate to connect to dynamixel
	// Verify that the port name is correct. This can be different on your system
	// Baudrate is set using the Dynamixel Wizard on the Dynamixel Servo directly
	const char* port = string("/dev/U2D2").c_str();
	int baudrate = 115200;
	vector<int> id_list = {1, 2, 3, 4};	
	
    // Create a DynamixelInterface object
	DynamixelPositionInterface dxl(port, baudrate, id_list);

    // Set the goal positions
	thread cmd_thread(send_command_looped, &dxl, id_list);

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
