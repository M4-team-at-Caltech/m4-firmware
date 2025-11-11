/**
 * @file M4Control.cpp
 * @brief Main control loop and command dispatch logic
 * 
 * Implements the real-time control loop that runs at CONTROL_LOOP_RATE Hz.
 * Handles state updates, command execution, and timing synchronization.
 * 
 * Functions:
 * - control_loop()
 * - control_step()
*/

#include "M4Base.h"

void M4Base::control_loop()
{
	printf("%s[INFO] Starting control loop...\n", CONSOLE_MSG_INFO);

	auto t0 = std::chrono::system_clock::now();
	auto t1 = std::chrono::system_clock::now();
	double dt, sleep_for;

    while(!quit)
    {
		if(!log_status && robot_state.size() >= LOG_BUFFER_SIZE)
		{
			robot_state.clear();
			state_updated = false;
		}

		control_step();

		// Sleep until next loop time
		t1 = std::chrono::system_clock::now();
		dt = get_duration_secs(t0, t1);
		sleep_for = (1.0/CONTROL_LOOP_RATE - dt) * 1.0e3;
		sleep_for = std::max(sleep_for,0.0);
		std::this_thread::sleep_for(std::chrono::milliseconds(int(sleep_for)));	
		t0 = std::chrono::system_clock::now();

    }
}

void M4Base::control_step()
{
	update_state();
	publish_state();

	// Execute commands
	if(!command_queue.empty() || command_active)
	{
		if(!command_active)
		{
			next_cmd = command_queue.pop();
			printf("%s[WARN]: Got command %s%s\n", CONSOLE_MSG_WARN, commands[next_cmd].c_str(), CONSOLE_DEFAULT);
		}

		switch(next_cmd)
		{
			case M4Commands::CMD_STAND:
				execute_stand();
				break;
			case M4Commands::CMD_CROUCH1:
				execute_crouch1();
				break;
			case M4Commands::CMD_CROUCH2:
				execute_crouch2();
				break;
			case M4Commands::CMD_SIT:
				execute_sit();
				break;
			case M4Commands::CMD_UAV:
				execute_uav();
				break;
			case M4Commands::CMD_MIP:
				execute_mip();
				break;
		}
	}

	else
	{
		execute_drive();
	}
}