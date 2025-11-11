#include <string>
#include <vector>
#include <atomic>
#include <csignal>
#include <yaml-cpp/yaml.h>
#include "M4Base.h"

// Global flag to indicate if a keyboard interrupt occurred
bool interrupted = false;

// Signal handler function to handle the SIGINT signal (Ctrl+C)
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Keyboard Interrupt received. Exiting gracefully..." << std::endl;
        interrupted = true;
    }
}

int main(int argc, char **argv)
{
	
	std::vector<int> active_joint_ids = {1,2,3,4,5,6,7,8};
	std::vector<int> active_wheel_ids = {9,10,11,12};

	M4Base m4(active_joint_ids, active_wheel_ids, true);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	m4.sit();
	std::cout<<"Sit complete!\n";
	while(m4.get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));
	std::cout<<"Sit wait complete!\n";
	
	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	M4Configurations current_configuration=M4Configurations::INIT;

	signal(SIGINT, signalHandler);

	float xvel_state = 0;
	float yvel_state = 0;
	
	float xvel_max = 1.0;
	float xvel_min = -1.0;
	float xvel_step = 0.1;
	
	float yvel_max = 1.0;
	float yvel_min = -1.0;
	float yvel_step = 0.1;

	float drive_mode = false;

    while(!interrupted)
    {
		current_configuration = m4.get_current_configuration();

		// Menu
        std::cout << "\n\nKeypress guide: " << std::endl;
		std::cout << "w : [DRIVE] Add lin vel" << std::endl;
        std::cout << "s : [DRIVE] Sub lin vel" << std::endl;
        std::cout << "d : [DRIVE] Add ang vel" << std::endl;
        std::cout << "a : [DRIVE] Sub ang vel" << std::endl;
    	std::cout << "z : [DRIVE] Stop moving" << std::endl;
        std::cout << "1 : [Mode Switch] STAND" << std::endl;
        std::cout << "2 : [Mode Switch] CROUCH 1" << std::endl;
		std::cout << "3 : [Mode Switch] CROUCH 2" << std::endl;
        std::cout << "U : [Mode Switch] UAV" << std::endl;
        std::cout << "q : Exit" << std::endl;

		// Status
		std::vector<std::string> mode_names = {"INIT", "STAND", "CROUCH 1", "CROUCH 2", "SIT", "UAV", "MIP"};
		std::cout << "Current Mode: [" << mode_names[current_configuration] << "]" << std::endl;
		if(drive_mode)
		{
			std::cout << "[DRIVE] Lin Vel: " << xvel_state << ", Ang Vel: " <<  yvel_state << std::endl;
		}

        char key;
		std::cout << "Enter Command (and press enter): ";
        std::cin >> key;
        std::cout << "Pressed: " << key << std::endl;

        switch (key)
        {
            case 'w': // [DRIVE] Forward
            {
				if(!drive_mode)
				{
					std::cout << "Not in [DRIVE] mode!" << std::endl;
					key = -1;
					break;
				}
				else
				{
					xvel_state += xvel_step;
					if(xvel_state > xvel_max)
					{
						std::cout << "[DRIVE] Max linear velocity reached!" << std::endl;
						xvel_state = xvel_max;
					}
				}
            }
            break;

            case 's': // [DRIVE] Backward
            {
				if(!drive_mode)
				{
					std::cout << "Not in [DRIVE] mode!" << std::endl;
					key = -1;
					break;
				}
				else
				{
					xvel_state -= xvel_step;
					if(xvel_state < xvel_min)
					{
						std::cout << "[DRIVE] Min linear velocity reached!" << std::endl;
						xvel_state = xvel_min;
					}
				}
            }
            break;

            case 'a': // [DRIVE] Left
            {
				if(!drive_mode)
				{
					std::cout << "Not in [DRIVE] mode!" << std::endl;
					key = -1;
					break;
				}
				else
				{
					yvel_state -= yvel_step;
					if(yvel_state < yvel_min)
					{
						std::cout << "[DRIVE] Min angular velocity reached!" << std::endl;
						yvel_state = yvel_min;
					}
				}
            }
			break;

			case 'd': // [DRIVE] Right
            {
				if(!drive_mode)
				{
					std::cout << "Not in [DRIVE] mode!" << std::endl;
					key = -1;
					break;
				}
				else
				{
					yvel_state += yvel_step;
					if(yvel_state > yvel_max)
					{
						std::cout << "[DRIVE] Max angular velocity reached!" << std::endl;
						yvel_state = yvel_max;
					}
				}
            }
			break;

            case 'z': // [DRIVE] Stop Moving
            {
				if(!drive_mode)
				{
					std::cout << "Not in [DRIVE] mode!" << std::endl;
					key = -1;
					break;
				}
				else
				{
					xvel_state = 0;
					yvel_state = 0;
				}
            }
            break;

            case '1': // [Mode Switch] Stand
            {
                if(current_configuration == M4Configurations::UAV)
				{
					char confirm;
					std::cout << "Switching out of UAV mode. Check that drone is not ARMED! \n\n Are you sure? (y/n): ";
					std::cin >> confirm;
					if(confirm != 'y' && confirm != 'Y')
					{
						std::cout << "Cancelling mode switching!" << std::endl;
						break;
					}
				}

				m4.stand();
            }
            break;

            case '2': // [Mode Switch] Crouch 1
            {
                if(current_configuration == M4Configurations::UAV)
				{
					char confirm;
					std::cout << "Switching out of UAV mode. Check that drone is not ARMED! \n\n Are you sure? (y/n): ";
					std::cin >> confirm;
					if(confirm != 'y' && confirm != 'Y')
					{
						std::cout << "Cancelling mode switching!" << std::endl;
						break;
					}
				}

				m4.crouch1();
            }
            break;

			case '3': // [Mode Switch] Crouch 2
            {
                if(current_configuration == M4Configurations::UAV)
				{
					char confirm;
					std::cout << "Switching out of UAV mode. Check that drone is not ARMED! \n\n Are you sure? (y/n): ";
					std::cin >> confirm;
					if(confirm != 'y' && confirm != 'Y')
					{
						std::cout << "Cancelling mode switching!" << std::endl;
						break;
					}
				}

				m4.crouch2();
            }
            break;

            case 'U': // [Mode Switch] UAV
            {
				drive_mode = false;
				m4.uav();
            }
            break;

            case 'q': // Exit
            {
				drive_mode = false;
                interrupted = true;
            }
            break;

            default:
            {
                std::cout << "Invalid key" << std::endl;
            }
        }


		// Do every loop
			if((current_configuration == M4Configurations::STAND) || (current_configuration == M4Configurations::CROUCH1) || (current_configuration == M4Configurations::CROUCH2))
			{
				drive_mode = true;
			}
			else
			{
				xvel_state = 0;
				yvel_state = 0;
			}
			m4.drive(xvel_state,yvel_state);
    }

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	m4.sit();
	while(m4.get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
