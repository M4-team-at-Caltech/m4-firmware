/**
 * @file M4Helpers.cpp
 * @brief Utility functions and accessors
 * 
 * Provides helper functions for joint limit enforcement, state queries,
 * and debugging output.
 * 
 * Functions:
 * - get_current_configuration()
 * - get_active_joint_positions()
 * - get_active_joint_names()
 * - get_active_wheel_velocities()
 * - get_active_wheel_names()
 * - RobotState::print()
 */

#include "M4Base.h"

M4Configurations M4Base::get_current_configuration()
{
	return current_configuration;
}

std::vector<float> M4Base::get_active_joint_positions(const std::vector<float>& joint_positions) 
{
    std::vector<float> active_joint_positions;
    
    // Using a map for efficient look-up of joint position based on joint ID
    std::unordered_map<int, float> joint_id_to_position;
    for (size_t i = 0; i < all_joint_ids.size(); ++i) {
        joint_id_to_position[all_joint_ids[i]] = joint_positions[i];
    }
    
    // Populating the active_joint_positions vector
    for (int id : active_joint_ids) {
        active_joint_positions.push_back(joint_id_to_position[id]);
    }
    
    return active_joint_positions;
}

std::vector<std::string> M4Base::get_active_joint_names() 
{
    std::vector<std::string> active_joint_names;
    
    // Using a map for efficient look-up of joint name based on joint ID
    std::unordered_map<int, std::string> joint_id_to_name;
    for (size_t i = 0; i < all_joint_ids.size(); ++i) {
        joint_id_to_name[all_joint_ids[i]] = all_joint_names[i];
    }
    
    // Populating the active_joint_names vector
    for (int id : active_joint_ids) {
        active_joint_names.push_back(joint_id_to_name[id]);
    }
    
    return active_joint_names;
}

std::vector<float> M4Base::get_active_wheel_velocities(const std::vector<float>& wheel_velocities) 
{
    std::vector<float> get_active_wheel_velocities;
    
    // Using a map for efficient look-up of wheel position based on wheel ID
    std::unordered_map<int, float> wheel_id_to_velocity;
    for (size_t i = 0; i < all_wheel_ids.size(); ++i) {
        wheel_id_to_velocity[all_wheel_ids[i]] = wheel_velocities[i];
    }
    
    // Populating the active_wheel_positions vector
    for (int id : active_wheel_ids) {
        get_active_wheel_velocities.push_back(wheel_id_to_velocity[id]);
    }
    
    return get_active_wheel_velocities;
}

std::vector<std::string> M4Base::get_active_wheel_names() 
{
    std::vector<std::string> active_wheel_names;
    
    // Using a map for efficient look-up of	wheel name based on wheel id
	std::unordered_map<int, std::string> wheel_id_to_name;
    for (size_t i = 0; i < all_wheel_ids.size(); ++i) {
        wheel_id_to_name[all_wheel_ids[i]] = all_wheel_names[i];
    }
    
    // Populating the active_wheel_names vector
    for (int id : active_wheel_ids) {
        active_wheel_names.push_back(wheel_id_to_name[id]);
    }
    
    return active_wheel_names;
}

void RobotState::print()
{
	std::cout << std::fixed << std::setprecision(4); 

	std::cout << "Robot state " << timestamp << std::endl;
	std::cout << "----------------------------------" << std::endl;
	
	std::cout << "Base state:" << std::endl;
	std::cout << "pb: "; for(int i=0; i<pb.size(); i++) std::cout << pb[i] << " "; std::cout << std::endl;
	std::cout << "qb: "; for(int i=0; i<qb.size(); i++) std::cout << qb[i] << " "; std::cout << std::endl;

	std::cout << "Wheel position:" << std::endl;
	std::cout << "pl: " << std::endl; 
	for(int i=0; i<pl.size(); i++) std::cout << std::setw(10) << pl[i][0] << " "; std::cout << std::endl;
	for(int i=0; i<pl.size(); i++) std::cout << std::setw(10) << pl[i][1] << " "; std::cout << std::endl;
	for(int i=0; i<pl.size(); i++) std::cout << std::setw(10) << pl[i][2] << " "; std::cout << std::endl;

	std::cout << "Joint state:" << std::endl;
	std::cout << "qj: "; for(int i=0; i<qj.size(); i++) std::cout << qj[i] << " "; std::cout << std::endl;
	std::cout << "qdotj: "; for(int i=0; i<qdotj.size(); i++) std::cout << qdotj[i] << " "; std::cout << std::endl;
	std::cout << "ij: "; for(int i=0; i<ij.size(); i++) std::cout << ij[i] << " "; std::cout << std::endl;
	std::cout << std::endl;

	std::cout << "Wheel state:" << std::endl;
	std::cout << "qw: "; for(int i=0; i<ql.size(); i++) std::cout << qw[i] << " "; std::cout << std::endl;
	std::cout << "qdotw: "; for(int i=0; i<qdotw.size(); i++) std::cout << qdotw[i] << " "; std::cout << std::endl;
	std::cout << "iw: "; for(int i=0; i<iw.size(); i++) std::cout << iw[i] << " "; std::cout << std::endl;
	std::cout << std::endl;

	std::cout << "----------------------------------" << std::endl;
}
