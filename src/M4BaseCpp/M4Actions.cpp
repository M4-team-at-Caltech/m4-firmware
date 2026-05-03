/**
 * @file M4Actions.cpp
 * @brief Discrete state-machine based motion primitives, and driving
 * 
 * Implements multi-step motion sequences for configuration changes and
 * position-based movements. Each function manages its own state machine.
 * 
 * Functions:
 * - execcute_drive()
 * - execute_stand()
 * - execute_crouch1()
 * - execute_crouch2()
 * - execute_sit()
 * - execute_uav()
 */

#include <math.h>
#include "M4Base.h"

void M4Base::execute_drive()
{
	// Differential drive controller
	float right_wheel_velocity, left_wheel_velocity = 0;
	{// MUTEX LOCK
		std::lock_guard<std::mutex> lock(drive_cmd);
		right_wheel_velocity = ((linear_velocity_cmd - 4 * angular_velocity_cmd * wheel_base / 2.0) / wheel_radius)*360/(2*M_PI);
		left_wheel_velocity = ((linear_velocity_cmd + 4 * angular_velocity_cmd * wheel_base / 2.0) / wheel_radius)*360/(2*M_PI);
	}// MUTEX UNLOCK

	std::vector<float> wheel_velocity_list = {left_wheel_velocity, right_wheel_velocity, left_wheel_velocity, right_wheel_velocity};
	wheel_velocity_cmd = get_active_wheel_velocities(wheel_velocity_list);
	wheel_velocity_cmd_ids = active_wheel_ids;
	dxl.write_velocity(wheel_velocity_cmd_ids, wheel_velocity_cmd);

}
 
void M4Base::execute_stand()
{
	switch(current_configuration)
	{
		// ---------------------------------------------------------
		// Already in STAND
		// ---------------------------------------------------------
		case M4Configurations::STAND:
			command_step = 0;
			command_active = false;
			return;

		// ---------------------------------------------------------
		// Transformation from CROUCH1/CROUCH2/SIT to STANDING
		// ---------------------------------------------------------
		case M4Configurations::CROUCH1:
			// Step 1: Move to Standing
			if(command_step == 0)
			{
			    dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				printf("[INFO] Stand Transformation complete!\n");
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}

			break;

		case M4Configurations::CROUCH2:
			// Step 1: Move to Standing
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 1)
			{
				printf("[INFO] Stand Transformation complete!\n");
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}
			
			break;

		case M4Configurations::INIT: // Initial configuration on robot powerup (should be close to SIT)
		case M4Configurations::SIT:
			// Step 1: Move to Standing
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Stand Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}
			break;

		// ---------------------------------------------------------
		// Transformation from UAV/MIP to STANDING
		// ---------------------------------------------------------
		case M4Configurations::MIP:
			//TODO: Not implemented.
			break;
		case M4Configurations::UAV:
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 4: Move to stand
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				command_step = 4;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, stand_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 4)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Stand Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::STAND;
			}

			break;
	}
}

void M4Base::execute_crouch1()
{
	switch(current_configuration)
	{
		// ---------------------------------------------------------
		// Already in CROUCH1
		// ---------------------------------------------------------
		case M4Configurations::CROUCH1:
			command_step = 0;
			command_active = false;
			return;

		// ---------------------------------------------------------
		// Transformation from CROUCH2/SIT to CROUCH1
		// ---------------------------------------------------------
		case M4Configurations::CROUCH2:
		case M4Configurations::SIT:
			// Step 1: Move to crouch1 position
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, crouch1_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH1;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from STAND to CROUCH1
		// ---------------------------------------------------------
		case M4Configurations::STAND:

			// Step 1: Move to crouch1 position
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, crouch1_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH1;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from UAV/MIP to CROUCH1
		// ---------------------------------------------------------
		case M4Configurations::MIP:
			//TODO: Not implemented yet
			break;
		case M4Configurations::UAV:
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 4: Move to crouch1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				dxl.write_position(active_joint_ids, crouch1_configuration);
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				gait_start = std::chrono::system_clock::now();
				command_step = 4;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.8 && command_step == 4)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH1;
			}

			break;
	}
	
}

void M4Base::execute_crouch2()
{
	switch(current_configuration)
	{
		// ---------------------------------------------------------
		// Already in CROUCH2
		// ---------------------------------------------------------
		case M4Configurations::CROUCH2:
			command_step = 0;
			command_active = false;
			return;

		// ---------------------------------------------------------
		// Transformation from CROUCH1 to CROUCH2
		// ---------------------------------------------------------
		case M4Configurations::CROUCH1:
			// Step 1: Move to crouch2 position
			if(command_step == 0)
			{
				dxl.write_position(active_joint_ids, crouch2_configuration);
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from SIT to CROUCH2
		// ---------------------------------------------------------
		case M4Configurations::SIT:
			// Step 1: Move to crouch2 position
			if(command_step == 0)
			{
				dxl.write_position(active_joint_ids, crouch2_configuration);
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from STAND to CROUCH2
		// ---------------------------------------------------------
		case M4Configurations::STAND:

			// Step 1: Move to crouch2 position
			if(command_step == 0)
			{
				dxl.write_position(active_joint_ids, crouch2_configuration);
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.8 && command_step == 1)
			{
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from MIP/UAV to CROUCH2
		// ---------------------------------------------------------
		case M4Configurations::MIP:
			//TODO: Not implemented
			break;
		case M4Configurations::UAV:
			// Step 1: Move to uav step2 position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 4: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 3)
			{
				command_step = 4;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 5: Move to crouch2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 4)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_in));
				dxl.write_position(active_joint_ids, crouch2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 5;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.2 && command_step == 5)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Crouch Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::CROUCH2;
			}

			break;
	}
}

void M4Base::execute_sit()
{
	switch(current_configuration)
	{
		// ---------------------------------------------------------
		// Already in SIT
		// ---------------------------------------------------------
		case M4Configurations::SIT:
			command_step = 0;
			command_active = false;
			return;

		// ---------------------------------------------------------
		// Transformation from INIT/STAND/CROUCH1/CROUCH2/MIP to SIT
		// (No intermediate steps required, can directly move to position)
		// ---------------------------------------------------------
		case M4Configurations::INIT:
		case M4Configurations::STAND:
		case M4Configurations::CROUCH1:
		case M4Configurations::CROUCH2:
		case M4Configurations::MIP:
			// Step 1: Move to sit position
			if(command_step == 0)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_step = 1;
				command_active = true;
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 1)
			{
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				printf("[INFO] Sit Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::SIT;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from UAV to SIT
		// ---------------------------------------------------------
		case M4Configurations::UAV:
			// Step 1: Move to uav step2 position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to sit position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] SIT Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::SIT;
			}
	}
}

void M4Base::execute_uav()
{
	switch(current_configuration)
	{
		// ---------------------------------------------------------
		// Already in UAV
		// ---------------------------------------------------------
		case M4Configurations::UAV:
			command_step = 0;
			command_active = false;
			return;
			
		// ---------------------------------------------------------
		// Transformation from CROUCH1 to UAV
		// ---------------------------------------------------------
		case M4Configurations::CROUCH1:
					// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.7 && command_step == 1)
			{
				command_step = 2;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to UAV step2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] UAV Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::UAV;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from STAND to UAV
		// ---------------------------------------------------------
		case M4Configurations::STAND:			// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 1.5 && command_step == 1)
			{
				command_step = 2;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to UAV step2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] UAV Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::UAV;
			}

			break;

		// ---------------------------------------------------------
		// Transformation from CROUCH2/SIT to UAV
		// ---------------------------------------------------------
		case M4Configurations::CROUCH2:
		case M4Configurations::SIT:
			// Step 1: Move to sit position
			if(command_step == 0)
			{
				command_step = 1;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities(transition_wheel_vel_out));
				dxl.write_position(active_joint_ids, sit_configuration);
				gait_start = std::chrono::system_clock::now();
				command_active = true;
			}

			// Step 2: Move to UAV step1 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 0.2 && command_step == 1)
			{
				command_step = 2;
				dxl.write_velocity(active_wheel_ids, get_active_wheel_velocities({0,0,0,0}));
				dxl.write_position(active_joint_ids, uav_step1_configuration);
				gait_start = std::chrono::system_clock::now();
			}

			// Step 3: Move to UAV step2 position
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 2)
			{
				command_step = 3;
				dxl.write_position(active_joint_ids, uav_step2_configuration);
				gait_start = std::chrono::system_clock::now();
			}
			
			// Complete
			if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 3)
			{
				printf("[INFO] UAV Transformation complete!\n");
				command_step = 0;
				command_active = false;
				current_configuration = M4Configurations::UAV;
			}

			break;
	}
}

void M4Base::execute_mip()
{
	//TODO: Not implemented
	// switch(current_configuration)
	// {
	// 	case M4Configurations::MIP:
	// 		command_step = 0;
	// 		command_active = false;
	// 		return;
	// 	case M4Configurations::CROUCH1:
	// 	case M4Configurations::CROUCH2:
	// 	case M4Configurations::STAND:
	// 	case M4Configurations::SIT:
	// 		// Step 1: Move to sit position
	// 		if(command_step == 0)
	// 		{
	// 			command_step = 1;
	// 			dxl.write_position(active_joint_ids, sit_configuration);
	// 			gait_start = std::chrono::system_clock::now();
	// 			command_active = true;
	// 		}

	// 		// Step 2: Move to MIP position
	// 		if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 3.5 && command_step == 1)
	// 		{
	// 			command_step = 2;
	// 			dxl.write_position(active_joint_ids, mip_configuration);
	// 			gait_start = std::chrono::system_clock::now();
	// 		}

	// 		// Complete
	// 		if(get_duration_secs(gait_start, std::chrono::system_clock::now()) >= 2 && command_step == 2)
	// 		{
	// 			printf("[INFO] MIP Transformation complete!\n");
	// 			command_step = 0;
	// 			command_active = false;
	// 			current_configuration = M4Configurations::MIP;
	// 		}

	// 		break;
	// }
}