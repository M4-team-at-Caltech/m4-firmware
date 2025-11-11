/**
 * @file M4CommandApi.cpp
 * @brief User-facing command API for M4
 * 
 * This file implements the high-level command interface that users call to control
 * the robot. Commands are queued and executed asynchronously by the control loop.
 * 
 * Functions:
 * - drive()
 * - crouch1()
 * - crouch2()
 * - stand()
 * - sit()
 * - uav()
 * - mip()
 */

#include "M4Base.h"

void M4Base::drive(float lin_vel_cmd, float ang_vel_cmd)
{
	{// MUTEX LOCK
		std::lock_guard<std::mutex> lock(drive_cmd);
		linear_velocity_cmd = lin_vel_cmd;
		angular_velocity_cmd = ang_vel_cmd;
	}// MUTEX UNLOCK
}

void M4Base::crouch1(){
	command_queue.push(M4Commands::CMD_CROUCH1);
	// Wait until it completes
	while(current_configuration != M4Configurations::CROUCH1){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::crouch2(){
	command_queue.push(M4Commands::CMD_CROUCH2);
	// Wait until it completes
	while(current_configuration != M4Configurations::CROUCH2){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::stand(){
    command_queue.push(M4Commands::CMD_STAND);
	// Wait until it completes
	while(current_configuration != M4Configurations::STAND){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::sit(){
    command_queue.push(M4Commands::CMD_SIT);
	// Wait until it completes
	while(current_configuration != M4Configurations::SIT){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::uav(){
    command_queue.push(M4Commands::CMD_UAV);
	// Wait until it completes
	while(current_configuration != M4Configurations::UAV){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}

void M4Base::mip(){
    command_queue.push(M4Commands::CMD_MIP);
	// Wait until it completes
	while(current_configuration != M4Configurations::MIP){std::this_thread::sleep_for(std::chrono::milliseconds(int(1000/CONTROL_LOOP_RATE)));}
}