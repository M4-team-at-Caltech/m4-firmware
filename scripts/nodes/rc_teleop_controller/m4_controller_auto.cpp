#include "m4_controller_node.h"

#include <yaml-cpp/yaml.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

// ─────────────────────────────────────────────────────────────────────────────
// Config loader
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::load_auto_config()
{
	try {
		std::string path = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
		YAML::Node cfg  = YAML::LoadFile(path);
		if (cfg["auto_mode"]) {
			odom_source_         = cfg["auto_mode"]["odom_source"].as<std::string>(odom_source_);
			odom_topic_          = cfg["auto_mode"]["odom_topic"].as<std::string>(odom_topic_);
			odom_timeout_s_      = cfg["auto_mode"]["odom_timeout_s"].as<double>(odom_timeout_s_);
			descent_rate_mps_    = cfg["auto_mode"]["odom_lost_descent_mps"].as<double>(descent_rate_mps_);
			arm_imu_wait_s_      = cfg["auto_mode"]["arm_imu_wait_s"].as<double>(arm_imu_wait_s_);
			arm_retry_window_s_  = cfg["auto_mode"]["arm_retry_window_s"].as<double>(arm_retry_window_s_);
			waypoint_timeout_s_  = cfg["auto_mode"]["waypoint_timeout_s"].as<double>(waypoint_timeout_s_);
			landing_descent_mps_ = cfg["auto_mode"]["landing_descent_mps"].as<double>(landing_descent_mps_);
			landing_z_stop_thr_  = cfg["auto_mode"]["landing_z_stop_thr"].as<double>(landing_z_stop_thr_);
			landing_z_stop_dur_  = cfg["auto_mode"]["landing_z_stop_dur"].as<double>(landing_z_stop_dur_);
		}
	} catch (const std::exception& e) {
		RCLCPP_WARN(this->get_logger(),
			"[m4_controller_node] Could not load auto_mode config: %s. Using defaults.", e.what());
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// AUTO mode callbacks
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::vehicle_land_detected_callback(
	const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	last_land_detected_ = *msg;
	has_land_detected_  = true;
}

void M4ControllerNode::mode_cmd_callback(const custom_msgs::msg::M4ModeCmd::SharedPtr msg)
{
	latest_mode_cmd_  = *msg;
	has_new_mode_cmd_ = true;
}

void M4ControllerNode::waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	latest_waypoint_        = *msg;
	last_waypoint_time_     = this->now();
	has_waypoint_           = true;
	last_position_setpoint_ = *msg;
}

void M4ControllerNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	latest_cmd_vel_    = *msg;
	last_cmd_vel_time_ = this->now();
	has_cmd_vel_       = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// PX4 offboard publish helpers
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::publish_offboard_control_mode(bool position_mode)
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
	msg.position     = position_mode;
	msg.velocity     = !position_mode;
	msg.acceleration = false;
	msg.attitude     = false;
	msg.body_rate    = false;
	offboard_control_mode_pub_->publish(msg);
}

// Publish position setpoint — transforms PoseStamped from its frame_id to ned_world via TF,
// then forwards ned_world coordinates directly as PX4 NED setpoint (EV mode).
void M4ControllerNode::publish_trajectory_setpoint(const geometry_msgs::msg::PoseStamped& wp)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	const std::string& src = wp.header.frame_id;
	if (!src.empty() && src != "ned_world") {
		try {
			geometry_msgs::msg::TransformStamped tf =
				tf_buffer_->lookupTransform("ned_world", src, tf2::TimePointZero);
			geometry_msgs::msg::PoseStamped wp_ned;
			tf2::doTransform(wp, wp_ned, tf);
			msg.position[0] = (float)wp_ned.pose.position.x;
			msg.position[1] = (float)wp_ned.pose.position.y;
			msg.position[2] = (float)wp_ned.pose.position.z;
		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
				"[AUTO] setpoint TF failed (%s) — skipping", ex.what());
			return;
		}
	} else {
		// frame_id empty or already ned_world — use coordinates directly
		msg.position[0] = (float)wp.pose.position.x;
		msg.position[1] = (float)wp.pose.position.y;
		msg.position[2] = (float)wp.pose.position.z;
	}

	msg.velocity[0] = msg.velocity[1] = msg.velocity[2]         = NaNf;
	msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = NaNf;
	msg.yaw = NaNf;
	trajectory_setpoint_pub_->publish(msg);
}

// Publish velocity-mode descent setpoint (DESCENDING state, odom lost)
void M4ControllerNode::publish_descent_setpoint()
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp     = this->get_clock()->now().nanoseconds() / 1000;
	msg.position[0]   = msg.position[1]   = msg.position[2]   = NaNf;
	msg.velocity[0]   = 0.0f;
	msg.velocity[1]   = 0.0f;
	msg.velocity[2]   = (float)descent_rate_mps_;  // Down: positive = descend (NED)
	msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = NaNf;
	msg.yaw           = NaNf;
	trajectory_setpoint_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// 10 Hz offboard heartbeat
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::offboard_heartbeat_callback()
{
	switch (offboard_state_) {
		case OffboardState::INACTIVE:
			return;

		case OffboardState::PRE_ARM:
			publish_offboard_control_mode(true);
			return;

		case OffboardState::TRACKING:
			publish_offboard_control_mode(true);
			publish_trajectory_setpoint(last_position_setpoint_);
			return;

		case OffboardState::DESCENDING:
			publish_offboard_control_mode(false);
			publish_descent_setpoint();
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				"[AUTO] DESCENDING — odom lost. Restore odom and resend mode_cmd=1 to resume.");
			return;

		case OffboardState::LANDING: {
			// Decrement Z in the cached frame (world ENU: z down = decrease toward 0)
			landing_setpoint_ps_.pose.position.z -= (float)(landing_descent_mps_ * 0.1);
			publish_offboard_control_mode(true);
			publish_trajectory_setpoint(landing_setpoint_ps_);
			return;
		}
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// Status helpers
// ─────────────────────────────────────────────────────────────────────────────
int32_t M4ControllerNode::config_to_int(M4Configurations cfg)
{
	switch (cfg) {
		case M4Configurations::STAND:   return 0;
		case M4Configurations::UAV:     return 1;
		case M4Configurations::CROUCH1: return 2;
		case M4Configurations::CROUCH2: return 4;
		case M4Configurations::SIT:     return -2;
		default:                        return -1;
	}
}

int32_t M4ControllerNode::offboard_state_to_int(OffboardState s)
{
	switch (s) {
		case OffboardState::INACTIVE:   return 0;
		case OffboardState::PRE_ARM:    return 1;
		case OffboardState::TRACKING:   return 2;
		case OffboardState::DESCENDING: return 3;
		case OffboardState::LANDING:    return 4;
		default:                        return 0;
	}
}

void M4ControllerNode::publish_mode_status()
{
	custom_msgs::msg::M4ModeStatus msg;
	msg.header.stamp     = this->get_clock()->now();
	msg.header.frame_id  = "";
	msg.configuration    = config_to_int(m4_configuration);
	msg.offboard_state   = offboard_state_to_int(offboard_state_);
	msg.auto_mode_active = auto_mode_active_;
	msg.odom_healthy     = odom_healthy_;
	msg.arm_pending      = arm_pending_;
	mode_status_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// Odometry utilities
// ─────────────────────────────────────────────────────────────────────────────
geometry_msgs::msg::PoseStamped M4ControllerNode::odom_to_posestamped() const
{
	geometry_msgs::msg::PoseStamped ps;
	ps.header.stamp    = last_odom_.header.stamp;
	ps.header.frame_id = "world";  // OdometryPublisher publishes in world (ENU) frame
	ps.pose            = last_odom_.pose.pose;
	const auto& q = ps.pose.orientation;
	if (std::abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1.0) > 0.01) {
		ps.pose.orientation = geometry_msgs::msg::Quaternion{};
		ps.pose.orientation.w = 1.0;
	}
	return ps;
}

// Convert ROS odometry (FLU world frame) to PX4 VehicleOdometry (NED).
// TF chain: ned_world ← world ← camera_init ← body ← base_link ← ned_base_link
// Requires static TFs: world→ned_world and base_link→ned_base_link (both 180° around X).
px4_msgs::msg::VehicleOdometry M4ControllerNode::convert_odom_flu_to_ned(
	const nav_msgs::msg::Odometry& ros_odom)
{
	px4_msgs::msg::VehicleOdometry px4_odom{};
	px4_odom.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
	px4_odom.timestamp_sample = px4_odom.timestamp;

	geometry_msgs::msg::TransformStamped tf_pose =
		tf_buffer_->lookupTransform("ned_world", "ned_base_link", tf2::TimePointZero);

	px4_odom.position[0] = (float)tf_pose.transform.translation.x;
	px4_odom.position[1] = (float)tf_pose.transform.translation.y;
	px4_odom.position[2] = (float)tf_pose.transform.translation.z;
	px4_odom.pose_frame  = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

	const auto& q = tf_pose.transform.rotation;
	px4_odom.q[0] = (float)q.w;
	px4_odom.q[1] = (float)q.x;
	px4_odom.q[2] = (float)q.y;
	px4_odom.q[3] = (float)q.z;

	// Linear velocity: world FLU → ned_world NED
	geometry_msgs::msg::Vector3Stamped vel_in, vel_out;
	vel_in.header = ros_odom.header;   // frame_id = "world"
	vel_in.vector = ros_odom.twist.twist.linear;
	tf_buffer_->transform(vel_in, vel_out, "ned_world");

	px4_odom.velocity[0]    = (float)vel_out.vector.x;
	px4_odom.velocity[1]    = (float)vel_out.vector.y;
	px4_odom.velocity[2]    = (float)vel_out.vector.z;
	px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

	// Angular velocity: base_link FLU → ned_base_link FRD
	geometry_msgs::msg::Vector3Stamped ang_in, ang_out;
	ang_in.header.frame_id = ros_odom.child_frame_id;  // "base_link"
	ang_in.header.stamp    = ros_odom.header.stamp;
	ang_in.vector          = ros_odom.twist.twist.angular;
	tf_buffer_->transform(ang_in, ang_out, "ned_base_link");

	px4_odom.angular_velocity[0] = (float)ang_out.vector.x;
	px4_odom.angular_velocity[1] = (float)ang_out.vector.y;
	px4_odom.angular_velocity[2] = (float)ang_out.vector.z;

	for (int i = 0; i < 3; i++) {
		px4_odom.position_variance[i]    = NaNf;
		px4_odom.orientation_variance[i] = NaNf;
		px4_odom.velocity_variance[i]    = NaNf;
	}
	px4_odom.quality = 1;
	return px4_odom;
}

// ─────────────────────────────────────────────────────────────────────────────
// LANDING: DISARM + re-queue ground mode_cmd for A2G transform
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::confirm_landing()
{
	send_arm_command(false);
	send_stabilized_mode_command();
	offboard_state_    = OffboardState::INACTIVE;
	landing_z_stopped_ = false;
	has_new_mode_cmd_ = true;
	latest_mode_cmd_  = pending_landing_mode_cmd_;
}

// ─────────────────────────────────────────────────────────────────────────────
// AUTO mode — process incoming M4ModeCmd
//
//  mode 0 = STAND   → ground
//  mode 1 = UAV     → aerial (arms PX4 if odom healthy)
//  mode 2 = CROUCH1 → ground
//  mode 4 = CROUCH2 → ground
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::process_mode_cmd(const custom_msgs::msg::M4ModeCmd& cmd)
{
	const int mode = cmd.mode;

	// ── mode 1: UAV (aerial) ──────────────────────────────────────────
	if (mode == 1) {
		arm_failed_ = false;  // new command clears previous failure so retry is allowed
		if (m4_configuration != M4Configurations::UAV) {
			RCLCPP_WARN(this->get_logger(), "[AUTO] Move to uav");
			#ifndef DRY_EXECUTION_MODE
				m4->drive(0, 0);
				m4->uav();
				m4_configuration = m4->get_current_configuration();
			#else
				m4_configuration = M4Configurations::UAV;
			#endif
		}
		auto_sub_state_    = AutoSubState::AERIAL;
		offboard_state_    = OffboardState::PRE_ARM;
		arm_pending_       = true;
		arm_pending_since_ = this->now();
		send_offboard_mode_command();
		RCLCPP_INFO(this->get_logger(),
			"[AUTO] mode=UAV — G2A transform triggered. "
			"Offboard heartbeat started. Waiting %.1fs for IMU to settle before ARM.",
			arm_imu_wait_s_);
		return;
	}

	// ── Ground modes (STAND / CROUCH1 / CROUCH2) ─────────────────────
	// If actively tracking, initiate controlled landing instead of immediate DISARM
	if (offboard_state_ == OffboardState::TRACKING) {
		pending_landing_mode_cmd_ = cmd;
		landing_setpoint_ps_      = last_position_setpoint_;  // preserves frame_id for TF
		landing_z_stopped_        = false;
		offboard_state_     = OffboardState::LANDING;
		RCLCPP_INFO(this->get_logger(),
			"[AUTO] Controlled landing initiated (target mode=%d after touchdown)", mode);
		return;
	}

	// Deactivate offboard if in other aerial states (PRE_ARM / DESCENDING)
	if (offboard_state_ != OffboardState::INACTIVE) {
		send_arm_command(false);
		send_stabilized_mode_command();
		offboard_state_ = OffboardState::INACTIVE;
	}

	// If coming from UAV config, wait for PX4 DISARM confirmation before A2G transform
	if (m4_configuration == M4Configurations::UAV) {
		if (pixhawk_state.arming_state ==
		    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
		{
			has_new_mode_cmd_ = true;  // re-queue: retry next loop tick
			latest_mode_cmd_  = cmd;
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
				"[AUTO] A2G transform pending — waiting for PX4 DISARM (arming_state=%d)",
				pixhawk_state.arming_state);
			return;
		}
		RCLCPP_WARN(this->get_logger(), "[AUTO] A2G transform — PX4 disarmed, move to stand");
		#ifndef DRY_EXECUTION_MODE
			m4->drive(0, 0);
			m4->stand();
			m4_configuration = m4->get_current_configuration();
		#else
			m4_configuration = M4Configurations::STAND;
		#endif
	}

	struct { int mode_val; DriveSwitchState state; } mapping[] = {
		{ 0, DriveSwitchState::STAND   },
		{ 2, DriveSwitchState::CROUCH1 },
		{ 4, DriveSwitchState::CROUCH2 },
	};

	for (const auto& m : mapping) {
		if (mode != m.mode_val) continue;
		for (const auto& handler : drive_mode_transition_handler) {
			if (handler.state == m.state &&
			    m4_configuration != handler.configuration)
			{
				RCLCPP_WARN(this->get_logger(), "[AUTO] %s", handler.message);
				#ifndef DRY_EXECUTION_MODE
					(m4->*handler.action)();
					m4_configuration = m4->get_current_configuration();
				#else
					m4_configuration = handler.configuration;
				#endif
			}
		}
		auto_sub_state_ = AutoSubState::GROUND;
		RCLCPP_INFO(this->get_logger(), "[AUTO] mode=%d applied", mode);
		return;
	}

	RCLCPP_WARN(this->get_logger(),
		"[AUTO] Unknown mode value: %d (valid: 0=STAND 1=UAV 2=CROUCH1 4=CROUCH2)", mode);
}

// ─────────────────────────────────────────────────────────────────────────────
// AUTO mode — ground execution (cmd_vel driven)
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::execute_auto_ground()
{
	const double cmd_vel_age = (this->now() - last_cmd_vel_time_).seconds();
	const bool   cmd_vel_ok  = has_cmd_vel_ && (cmd_vel_age < odom_timeout_s_);

	#ifndef DRY_EXECUTION_MODE
		if (cmd_vel_ok) {
			m4->drive(latest_cmd_vel_.linear.x, -latest_cmd_vel_.angular.z);
		} else {
			m4->drive(0.0f, 0.0f);
			if (has_cmd_vel_)
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"[AUTO] cmd_vel timeout (%.2fs) — holding zero", cmd_vel_age);
		}
	#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// AUTO mode — aerial execution
// Heartbeat + setpoint publishing handled by offboard_timer_.
// This function only manages state transitions and setpoint cache updates.
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::execute_auto_aerial()
{
	// ── Phase 1: IMU settle wait ──────────────────────────────────────────
	if (arm_pending_) {
		const double elapsed   = (this->now() - arm_pending_since_).seconds();
		const double remaining = arm_imu_wait_s_ - elapsed;
		if (remaining > 0.0) {
			RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
				"[AUTO] IMU settling... %.1fs remaining", remaining);
			return;
		}
		arm_pending_ = false;
		if (!odom_healthy_) {
			RCLCPP_ERROR(this->get_logger(),
				"[AUTO] IMU settle complete but odom not healthy — ARM blocked → INACTIVE");
			offboard_state_ = OffboardState::INACTIVE;
			return;
		}
		last_position_setpoint_ = odom_to_posestamped();
		send_offboard_mode_command();
		send_arm_command(true);
		arm_retry_active_ = true;
		arm_retry_since_  = this->now();
		RCLCPP_INFO(this->get_logger(),
			"[AUTO] IMU settle complete — ARM sent, retry window %.1fs", arm_retry_window_s_);
		return;
	}

	// ── Phase 2: ARM confirmation + retry window ──────────────────────────
	if (arm_retry_active_) {
		if (pixhawk_state.arming_state ==
		    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
			arm_retry_active_ = false;
			offboard_state_   = OffboardState::TRACKING;
			RCLCPP_INFO(this->get_logger(),
				"[AUTO] ARM confirmed → TRACKING (hover: %.2f, %.2f, %.2f FLU)",
				last_position_setpoint_.pose.position.x,
				last_position_setpoint_.pose.position.y,
				last_position_setpoint_.pose.position.z);
			return;
		}
		const double elapsed = (this->now() - arm_retry_since_).seconds();
		if (elapsed >= arm_retry_window_s_) {
			arm_retry_active_ = false;
			arm_failed_       = true;   // prevent INACTIVE path from auto-retrying
			offboard_state_   = OffboardState::INACTIVE;
			RCLCPP_ERROR(this->get_logger(),
				"[AUTO] ARM failed after %.1fs retry window → INACTIVE", arm_retry_window_s_);
			return;
		}
		send_arm_command(true);
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
			"[AUTO] ARM not confirmed (%.1f/%.1fs) — retrying...", elapsed, arm_retry_window_s_);
		return;
	}

	// ── INACTIVE: odom-recovery re-arm ────────────────────────────────────
	if (offboard_state_ == OffboardState::INACTIVE) {
		// has_new_mode_cmd_ means a ground cmd is pending disarm confirmation
		// (set by the A2G re-queue path) — do NOT re-arm in this case
		if (has_new_mode_cmd_) return;
		// arm_failed_ means the retry window already expired — do NOT auto-retry;
		// operator must resend mode_cmd=1 to try again
		if (arm_failed_) return;

		if (odom_healthy_) {
			last_position_setpoint_ = odom_to_posestamped();
			send_offboard_mode_command();
			send_arm_command(true);
			arm_retry_active_ = true;
			arm_retry_since_  = this->now();
			offboard_state_   = OffboardState::PRE_ARM;
			RCLCPP_INFO(this->get_logger(), "[AUTO] odom recovered — ARM retry starting");
		} else {
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				"[AUTO] Waiting for odometry before arming... topic: %s",
				odom_topic_.c_str());
		}
		return;
	}

	// ── DESCENDING: heartbeat handles publishing ───────────────────────────
	if (offboard_state_ == OffboardState::DESCENDING) {
		return;
	}

	// ── LANDING: check touchdown ───────────────────────────────────────────
	if (offboard_state_ == OffboardState::LANDING) {
		if (has_land_detected_ && last_land_detected_.landed) {
			RCLCPP_INFO(this->get_logger(),
				"[AUTO] Touchdown confirmed (vehicle_land_detected) → DISARM + A2G");
			confirm_landing();
			return;
		}
		if (odom_healthy_) {
			const double z_vel = std::abs(last_odom_.twist.twist.linear.z);
			if (z_vel < landing_z_stop_thr_) {
				if (!landing_z_stopped_) {
					landing_z_stopped_    = true;
					landing_z_stop_start_ = this->now();
				} else if ((this->now() - landing_z_stop_start_).seconds() >= landing_z_stop_dur_) {
					RCLCPP_INFO(this->get_logger(),
						"[AUTO] Touchdown confirmed (odom Z stop fallback) → DISARM + A2G");
					confirm_landing();
				}
			} else {
				landing_z_stopped_ = false;
			}
		}
		return;
	}

	// ── TRACKING: update position setpoint cache for the heartbeat timer ──
	const double wp_age = has_waypoint_
		? (this->now() - last_waypoint_time_).seconds()
		: std::numeric_limits<double>::max();

	if (wp_age < waypoint_timeout_s_) {
		last_position_setpoint_ = latest_waypoint_;
	} else {
		if (odom_healthy_) {
			last_position_setpoint_ = odom_to_posestamped();
			if (!has_waypoint_) {
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"[AUTO] No waypoint received — hovering at odom position");
			} else {
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"[AUTO] Waypoint timeout (%.2fs > %.2fs) — hovering at odom",
					wp_age, waypoint_timeout_s_);
			}
		}
		// !odom_healthy_ → loop() odom tick triggers TRACKING → DESCENDING
	}
}
