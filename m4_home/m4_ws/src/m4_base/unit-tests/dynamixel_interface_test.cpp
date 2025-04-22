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

void send_command_looped(DynamixelInterface *dxl, vector<int> id_list, vector<int> wheel_list)
{
	while (!stopFlag)
	{	
		float p1=0, p2=0, v1=0;

		cout << "Enter inputs: ";
		cin >> p1 >> p2 >> v1;
		cout << "Sending inputs...\n";

		vector<float> goal_positions = {p1, p2}; 
		dxl->write_position(id_list, goal_positions);
		
		vector<float> goal_velocities = {v1};
		dxl->write_velocity(wheel_list, goal_velocities);

		cout << "Press q to quit" << endl;
		char c = getchar();
		if(c == 'q' || c == 'Q')
		{
            stopFlag = true;
        }
    }
}

int main(int argc, char *argv[])
{
	// Port and baudrate to connect to dynamixel
	// Verify that the port name is correct. This can be different on your system
	// Baudrate is set using the Dynamixel Wizard on the Dynamixel Servo directly
	const char* port = string("/dev/U2D2").c_str();
	int baudrate = 4500000;
	vector<int> id_list = {3,4};	
	vector<int> wheel_list = {10};
	
    // Create a DynamixelInterface object
	DynamixelInterface dxl(port, baudrate, id_list, wheel_list);

    // Set the goal positions
	send_command_looped(&dxl, id_list, wheel_list);

	cout << endl;
	return 0;
}
