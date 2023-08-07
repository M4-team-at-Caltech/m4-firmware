#include <vector>
#include <iostream>
#include "m4_base.h"

int main(int argc, char *argv[])
{
	std::vector<int> active_joint_ids = {1, 2, 3, 4};
	M4Base m4(active_joint_ids, true);

	std::vector<float> goal_positions1 = {0, 60, 0, 60}; 
	std::vector<float> goal_positions2 = {90, 60, 90, 60}; 
	std::vector<float> goal_positions3 = {0, 60, 0, 60}; 
	std::vector<float> goal_positions4 = {0, 0, 0, 0};

	m4.custom_joint_pos(active_joint_ids, goal_positions1);
	while(!m4.is_moving()){std::this_thread::sleep_for(std::chrono::milliseconds(100));}
	
	m4.custom_joint_pos(active_joint_ids, goal_positions2);
	while(!m4.is_moving()){std::this_thread::sleep_for(std::chrono::milliseconds(100));}
	
	m4.custom_joint_pos(active_joint_ids, goal_positions3);
	while(!m4.is_moving()){std::this_thread::sleep_for(std::chrono::milliseconds(100));}
	
	m4.custom_joint_pos(active_joint_ids, goal_positions4);
	while(!m4.is_moving()){std::this_thread::sleep_for(std::chrono::milliseconds(100));}

	std::cout << "Movement Complete!" << std::endl;

    return 0;
}