#include <CobraRoot.h>
#include <iostream>
#include <csignal>

// Global flag to indicate if a keyboard interrupt occurred
bool interrupted = false;

// Signal handler function to handle the SIGINT signal (Ctrl+C)
void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "Keyboard Interrupt received. Exiting gracefully..." << std::endl;
        interrupted = true;
    }
}

int main()
{
        CobraRoot cobra({1, 2, 3, 4, 5, 6, 7, 8, 9, 11});
	cobra.set_log_status(true);
	while(!interrupted)
	{

	        std::this_thread::sleep_for(std::chrono::seconds(1));
		if(cobra.robot_state_available())
		{
			RobotState state = cobra.get_state();
			std::cout << "q: "; for(int i=0; i<11; i++) std::cout << state.q[i] << " "; std::cout << std::endl;
			std::cout << "qdot: "; for(int i=0; i<11; i++) std::cout << state.qdot[i] << " "; std::cout << std::endl;
			std::cout << "current: "; for(int i=0; i<11; i++) std::cout << state.current[i] << " "; std::cout << std::endl;
			std::cout << std::endl;
		}
	}

	cobra.stop();
	return 0;
}
