#include <vector>
#include <iostream>
#include "m4_base.h"

int main(int argc, char *argv[])
{
	std::vector<int> active_joint_ids = {1, 2, 3, 4};
	M4Base m4(active_joint_ids, true);

	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.crouch();
	while(m4.get_current_configuration() != M4Configurations::CROUCH) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.uav();
	while(m4.get_current_configuration() != M4Configurations::UAV) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.mip();
	while(m4.get_current_configuration() != M4Configurations::MIP) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	m4.stand();
	while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));

	std::cout << "Sequence complete!\n";


	// std::cout << "\n\n Init complete \n\n";

	// std::vector<float> goal_positions1 = {0, 60, 0, 60}; 
	// std::vector<float> goal_positions2 = {90, 60, 90, 60}; 
	// std::vector<float> goal_positions3 = {0, 60, 0, 60}; 
	// std::vector<float> goal_positions4 = {0, 0, 0, 0};

	// std::cout << "First goal pos \n";

	// m4.custom_joint_pos(active_joint_ids, goal_positions1, 5);
	// std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	
	// std::cout << "\nSecond goal pos \n";

	// m4.custom_joint_pos(active_joint_ids, goal_positions2, 5);
	// std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	
	// std::cout << "\nThird goal pos \n";

	// m4.custom_joint_pos(active_joint_ids, goal_positions3, 5);
	// std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	
	// std::cout << "\nFourth goal pos \n";

	// m4.custom_joint_pos(active_joint_ids, goal_positions4, 5);
	// std::this_thread::sleep_for(std::chrono::milliseconds(5000));

	// std::cout << "Movement Complete!" << std::endl;

    return 0;
}