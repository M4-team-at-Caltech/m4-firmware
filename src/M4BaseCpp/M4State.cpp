/**
 * @file M4State.cpp
 * @brief State estimation and kinematics computation
 * 
 * Reads hardware state and computes derived quantities including base pose,
 * forward kinematics.
 * 
 * Functions:
 * - update_state()
 * - update_base_state()
 * - update_kinematics()
 */

#include "M4Base.h"
using namespace std;
void M4Base::update_state()
{
	// DynamixelStatus status = dxl.read_status();
	DynamixelStatus status = hw_read_status();
	RobotState r;
	r.timestamp = status.timestamp;
	r.qj = status.qj;
	r.qdotj = status.qdotj;
	r.ij = status.ij;
	r.qw = status.qw;
	r.qdotw = status.qdotw;
	r.iw = status.iw;
	
	// Update base state
	update_base_state(r);

	// Update kinematics
	update_kinematics(r);

	robot_state.push_back(r);
	state_updated = true;
}

void M4Base::update_base_state(RobotState& r)
{
	r.pb = std::vector<float>(3, 0.0);
	r.qb = std::vector<float>(4, 0.0); r.qb[3] = 1.0; // w component
}

void M4Base::update_kinematics(RobotState& r)
{
	//TODO: Implement kinematics calculation for wheel poses. Nominaly zero pose for now.
	r.pl.assign(4, std::vector<float>(3, 0.0f));
	r.ql.assign(4, std::vector<float>(4, 0.0f));
	r.ql[0][3] = 1.0f; r.ql[1][3] = 1.0f; r.ql[2][3] = 1.0f; r.ql[3][3] = 1.0f;
}

bool M4Base::state_available()
{
	return state_updated;
}

RobotState M4Base::get_state()
{
	state_updated = false;
	return robot_state.back();
}