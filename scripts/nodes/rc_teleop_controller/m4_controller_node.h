#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <cmath>
#include <limits>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <custom_msgs/msg/m4_mode_cmd.hpp>
#include <custom_msgs/msg/m4_mode_status.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "src/RCcontroller.h"
#include "M4Base.h"

inline constexpr float NaNf = std::numeric_limits<float>::quiet_NaN();

// ─────────────────────────────────────────────────────────────────────────────
// Transition handler structs
// ─────────────────────────────────────────────────────────────────────────────
struct DriveModeTransitionHandler
{
	DriveSwitchState state;
	M4Configurations configuration;
	const char* message;
	void (M4Base::*action)();
};

struct UAVModeTransitionHandler
{
	UAVSwitchState state;
	M4Configurations configuration;
	const char* message;
	void (M4Base::*action)();
};

// ─────────────────────────────────────────────────────────────────────────────
// M4ControllerNode
// ─────────────────────────────────────────────────────────────────────────────
class M4ControllerNode : public rclcpp::Node
{
public:
	M4ControllerNode(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids);
	~M4ControllerNode();

private:
	// ── Hardware state ──
	M4Configurations m4_configuration;
	M4Base *m4;
	std::vector<DriveModeTransitionHandler> drive_mode_transition_handler;
	std::vector<UAVModeTransitionHandler>   uav_mode_transition_handler;

	RCController rc_controller;
	RCstate      rc_state;

	rclcpp::TimerBase::SharedPtr timer_;           // 50 ms main loop
	rclcpp::TimerBase::SharedPtr offboard_timer_;  // 10 Hz offboard heartbeat

	px4_msgs::msg::VehicleStatus pixhawk_state;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr state_subscriber_;

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr        vehicle_command_publisher_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr      vehicle_odom_pub_;
	rclcpp::Publisher<custom_msgs::msg::M4ModeStatus>::SharedPtr      mode_status_pub_;

	std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	// ── AUTO mode: subscribers ──
	rclcpp::Subscription<custom_msgs::msg::M4ModeCmd>::SharedPtr      auto_mode_cmd_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  auto_waypoint_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr        auto_cmd_vel_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;

	// ── AUTO mode: config ──
	std::string odom_source_{"fastlio"};
	std::string odom_topic_{"/localization/odom"};
	double      odom_timeout_s_{0.5};
	double      descent_rate_mps_{0.3};   // downward speed when odom is lost (DESCENDING)
	double      arm_imu_wait_s_{3.0};     // wait after G2A transform before ARM (IMU settle)
	double      waypoint_timeout_s_{2.0}; // aerial waypoint staleness threshold
	double      landing_descent_mps_{0.3};  // controlled descent rate during LANDING (m/s)
	double      landing_z_stop_thr_{0.05};  // odom Z velocity threshold to detect touchdown
	double      landing_z_stop_dur_{1.0};   // seconds Z must stay below threshold to confirm touchdown

	// ── AUTO mode: high-level state ──
	bool            auto_mode_active_{false};
	AutoSwitchState prev_auto_switch_{AutoSwitchState::MANUAL};

	enum class AutoSubState { IDLE, GROUND, AERIAL };
	AutoSubState auto_sub_state_{AutoSubState::IDLE};

	// ── UAV ARM: IMU settle wait + confirmation retry ──
	bool         arm_pending_{false};
	rclcpp::Time arm_pending_since_{0, 0, RCL_ROS_TIME};
	bool         arm_retry_active_{false};
	rclcpp::Time arm_retry_since_{0, 0, RCL_ROS_TIME};
	double       arm_retry_window_s_{3.0};

	// ── LANDING state ──
	custom_msgs::msg::M4ModeCmd  pending_landing_mode_cmd_{};
	float        landing_setpoint_x_{0.0f};
	float        landing_setpoint_y_{0.0f};
	float        landing_setpoint_z_{0.0f};
	bool         landing_z_stopped_{false};
	rclcpp::Time landing_z_stop_start_{0, 0, RCL_ROS_TIME};

	bool         has_land_detected_{false};
	px4_msgs::msg::VehicleLandDetected last_land_detected_{};
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_sub_;

	// ── Offboard state machine ──
	//   INACTIVE   : heartbeat not sent, PX4 not in offboard
	//   PRE_ARM    : IMU settle wait + ARM retry window — heartbeat sent (position=true)
	//   TRACKING   : position control, publishes setpoint from waypoint/odom
	//   DESCENDING : velocity control descent; entered on odom loss mid-flight
	//   LANDING    : position control with decreasing Z; confirms touchdown then A2G
	enum class OffboardState { INACTIVE, PRE_ARM, TRACKING, DESCENDING, LANDING };
	OffboardState offboard_state_{OffboardState::INACTIVE};

	// ── Odom health ──
	bool         odom_healthy_{false};
	rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
	nav_msgs::msg::Odometry last_odom_;

	// ── Cached AUTO inputs ──
	bool has_new_mode_cmd_{false};
	custom_msgs::msg::M4ModeCmd latest_mode_cmd_;

	bool         has_waypoint_{false};
	rclcpp::Time last_waypoint_time_{0, 0, RCL_ROS_TIME};
	geometry_msgs::msg::PoseStamped latest_waypoint_;
	geometry_msgs::msg::PoseStamped last_position_setpoint_;  // cache for heartbeat timer

	bool         has_cmd_vel_{false};
	rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
	geometry_msgs::msg::Twist latest_cmd_vel_;

	// ── Core methods (m4_controller_node.cpp) ──
	void init_controller();
	void send_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	void send_arm_command(bool arm);
	void send_offboard_mode_command();
	void send_stabilized_mode_command();
	void state_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void loop();

	// ── AUTO mode methods (m4_controller_auto.cpp) ──
	void load_auto_config();
	void vehicle_land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	void mode_cmd_callback(const custom_msgs::msg::M4ModeCmd::SharedPtr msg);
	void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void confirm_landing();
	void publish_offboard_control_mode(bool position_mode);
	void publish_trajectory_setpoint(const geometry_msgs::msg::PoseStamped& wp);
	void publish_descent_setpoint();
	void offboard_heartbeat_callback();
	static int32_t config_to_int(M4Configurations cfg);
	static int32_t offboard_state_to_int(OffboardState s);
	void publish_mode_status();
	geometry_msgs::msg::PoseStamped odom_to_posestamped() const;
	px4_msgs::msg::VehicleOdometry convert_odom_flu_to_ned(const nav_msgs::msg::Odometry& ros_odom);
	void process_mode_cmd(const custom_msgs::msg::M4ModeCmd& cmd);
	void execute_auto_ground();
	void execute_auto_aerial();
};
