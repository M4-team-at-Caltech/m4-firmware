/**
 * @file M4HwBridge.cpp
 * @brief Hardware abstraction wrappers for joint/wheel I/O
 *
 * hw_write_position / hw_write_velocity / hw_read_status route calls to either:
 *   - DynamixelInterface (real hardware, default)
 *   - ROS2 publishers to Gazebo bridge topics (SIMULATION build flag)
 *
 * Joint ID → topic mapping (from robot_config.yaml joint layout):
 *   id 1,3,5,7 (Frontal/hip)   → /Drone1/servo_hip_{0,1,2,3}  (radians)
 *   id 2,4,6,8 (Saggital/leg)  → /Drone1/servo_leg_{0,1,2,3}  (radians)
 *
 * Wheel ID → topic mapping:
 *   id 9,10,11,12               → /Drone1/vel_wheel_{0,1,2,3}  (rad/s)
 *
 * Velocity unit note: M4Actions passes wheel velocities in deg/s (derived from
 * differential drive kinematics). Converted to rad/s for Gazebo (× π/180).
 * Transition wheel velocities from yaml are in RPM; these pass through the same
 * conversion — verify with hardware calibration if behaviour differs.
 */

#include "M4Base.h"

#ifdef SIMULATION
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#endif

// ─────────────────────────────────────────────────────────────────────────────
void M4Base::hw_write_position(const std::vector<int>& ids, const std::vector<float>& pos)
{
#ifdef SIMULATION
    for (size_t i = 0; i < ids.size() && i < pos.size(); i++) {
        const int id = ids[i];   // 1–8
        const float rad = pos[i] * static_cast<float>(M_PI) / 180.0f;

        std_msgs::msg::Float64 msg;
        msg.data = static_cast<double>(rad);

        if (id % 2 == 1) {
            // Odd IDs: Frontal (hip) joints — id 1,3,5,7 → hip 0,1,2,3
            const int idx = (id - 1) / 2;
            if (idx >= 0 && idx < 4) sim_hip_pubs_[idx]->publish(msg);
        } else {
            // Even IDs: Saggital (leg) joints — id 2,4,6,8 → leg 0,1,2,3
            const int idx = id / 2 - 1;
            if (idx >= 0 && idx < 4) sim_leg_pubs_[idx]->publish(msg);
        }

        // Update synthetic state for hw_read_status()
        if (i < sim_joint_positions_.size())
            sim_joint_positions_[i] = pos[i];
    }
#else
    dxl.write_position(ids, pos);
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
void M4Base::hw_write_velocity(const std::vector<int>& ids, const std::vector<float>& vel)
{
#ifdef SIMULATION
    for (size_t i = 0; i < ids.size() && i < vel.size(); i++) {
        const int id = ids[i];   // 9–12
        const int idx = id - 9;  // 0–3
        if (idx < 0 || idx >= 4) continue;

        std_msgs::msg::Float64 msg;
        msg.data = static_cast<double>(vel[i]) * M_PI / 180.0;  // deg/s → rad/s
        sim_wheel_pubs_[idx]->publish(msg);

        if (i < sim_wheel_velocities_.size())
            sim_wheel_velocities_[i] = vel[i];
    }
#else
    dxl.write_velocity(ids, vel);
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
DynamixelStatus M4Base::hw_read_status()
{
#ifdef SIMULATION
    // Return synthetic status using last commanded values (open-loop).
    DynamixelStatus status{};
    status.timestamp = 0.0f;
    status.qj    = sim_joint_positions_;
    status.qdotj.assign(sim_joint_positions_.size(), 0.0f);
    status.ij.assign(sim_joint_positions_.size(), 0.0f);
    status.qw    = std::vector<float>(sim_wheel_velocities_.size(), 0.0f);
    status.qdotw = sim_wheel_velocities_;
    status.iw.assign(sim_wheel_velocities_.size(), 0.0f);
    return status;
#else
    return dxl.read_status();
#endif
}
