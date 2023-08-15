#include <vector>
#include <iostream>
#include "m4_base.h"

int main(int argc, char *argv[])
{
	std::vector<int> active_joint_ids = {1, 2, 3, 4, 5, 6, 7, 8};
	M4Base m4(active_joint_ids, true);

	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// m4.sit();
	// while(m4.get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// m4.stand();
	// while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// m4.crouch();
	// while(m4.get_current_configuration() != M4Configurations::CROUCH) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// m4.stand();
	// while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// m4.uav();
	// while(m4.get_current_configuration() != M4Configurations::UAV) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	// m4.stand();
	// while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.uav();
	while(m4.get_current_configuration() != M4Configurations::UAV) std::this_thread::sleep_for(std::chrono::milliseconds(100));
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	// m4.mip();
	// while(m4.get_current_configuration() != M4Configurations::MIP) std::this_thread::sleep_for(std::chrono::milliseconds(100));
	// std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	std::cout << "Sequence complete!\n";

    return 0;
}
