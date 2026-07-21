#include "m4_controller_node.h"

#include <yaml-cpp/yaml.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

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
			landing_lowthr_frames_    = cfg["auto_mode"]["landing_lowthr_frames"].as<int>(landing_lowthr_frames_);
			landing_disarm_timeout_s_ = cfg["auto_mode"]["landing_disarm_timeout_s"].as<double>(landing_disarm_timeout_s_);
			landing_detect_vz_        = cfg["auto_mode"]["landing_detect_vz"].as<double>(landing_detect_vz_);
			landing_force_disarm_s_   = cfg["auto_mode"]["landing_force_disarm_s"].as<double>(landing_force_disarm_s_);
			setpoint_source_     = cfg["auto_mode"]["setpoint_source"].as<std::string>(setpoint_source_);
			localization_mode_   = cfg["auto_mode"]["localization_mode"].as<std::string>(localization_mode_);
		}
	} catch (const std::exception& e) {
		RCLCPP_WARN(this->get_logger(),
			"[m4_controller_node] Could not load auto_mode config: %s. Using defaults.", e.what());
	}
	if (setpoint_source_ != "pose" && setpoint_source_ != "pva") {
		RCLCPP_WARN(this->get_logger(),
			"[AUTO] invalid setpoint_source '%s' -> falling back to 'pose'", setpoint_source_.c_str());
		setpoint_source_ = "pose";
	}
	pva_input_ = (setpoint_source_ == "pva");
	if (localization_mode_ != "localization" && localization_mode_ != "relocalization") {
		RCLCPP_WARN(this->get_logger(),
			"[AUTO] invalid localization_mode '%s' -> falling back to 'localization'",
			localization_mode_.c_str());
		localization_mode_ = "localization";
	}
	relocal_mode_ = (localization_mode_ == "relocalization");
	RCLCPP_INFO(this->get_logger(), "[AUTO] setpoint_source = %s, localization_mode = %s (NED anchor: %s)",
		setpoint_source_.c_str(), localization_mode_.c_str(),
		relocal_mode_ ? "ned_map" : "ned_world");
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

void M4ControllerNode::trajectory_setpoint_pva_callback(
	const custom_msgs::msg::M4TrajectorySetpoint::SharedPtr msg)
{
	latest_pva_     = *msg;
	last_pva_time_  = this->now();
	has_pva_        = true;
	// also feed the pose cache so landing / odom-lost fallbacks keep a valid position
	// target (frame_id preserved for TF), and so a pva->pose handoff is seamless.
	last_position_setpoint_.header      = msg->header;
	last_position_setpoint_.pose.position    = msg->position;
	last_position_setpoint_.pose.orientation = msg->orientation;
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
void M4ControllerNode::publish_offboard_control_mode(bool position_mode, bool pva)
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp    = this->get_clock()->now().nanoseconds() / 1000;
	// PVA: position control with velocity + acceleration feed-forward (all enabled).
	// Otherwise position_mode selects pure position (true) or velocity (false) control.
	msg.position     = pva ? true : position_mode;
	msg.velocity     = pva ? true : !position_mode;
	msg.acceleration = pva;
	msg.attitude     = false;
	msg.body_rate    = false;
	offboard_control_mode_pub_->publish(msg);
}

// Convert a desired heading quaternion (already expressed in ned_world, but still
// FLU-body convention as it came from the planner) into a PX4 NED yaw, matching the
// odom convention (FRD-in-NED). convert_odom_flu_to_ned() obtains odom q through the
// full chain ned_world<-world<-...<-base_link<-ned_base_link, whose final static link
// (base_link->ned_base_link, 180 deg about X) flips FLU body to FRD. We apply the same
// 180-deg-about-X body correction on the right so the streamed yaw uses the identical
// frame as the odom feedback. Returns NaN (heading uncontrolled) for a null quaternion.
float M4ControllerNode::setpoint_yaw_ned(const geometry_msgs::msg::Quaternion& ori_nedworld) const
{
	tf2::Quaternion q_flu;
	tf2::fromMsg(ori_nedworld, q_flu);
	if (q_flu.length2() < 1e-6) return NaNf;  // no orientation provided
	tf2::Quaternion q_x180;
	q_x180.setRPY(M_PI, 0.0, 0.0);            // FLU body -> FRD body (same as ned_base_link link)
	const tf2::Quaternion q_frd = q_flu * q_x180;
	double roll, pitch, yaw;
	tf2::Matrix3x3(q_frd).getRPY(roll, pitch, yaw);
	return (float)yaw;
}

// NED anchor frame for setpoint conversion: ned_map (relocalization, scan_lock
// global) or ned_world (localization, pure fastlio). map→ned_map is a static TF
// (added in m4_relocalization.launch.py), so lookupTransform resolves without
// depending on scan_lock convergence — same TF path as the ned_world case.
const char* M4ControllerNode::ned_anchor_frame() const
{
	return relocal_mode_ ? "ned_map" : "ned_world";
}

// Publish position setpoint — transforms PoseStamped from its frame_id to ned_world via TF,
// then forwards ned_world coordinates directly as PX4 NED setpoint (EV mode). Heading
// (yaw) is taken from the waypoint orientation (F1) so the planner controls facing.
void M4ControllerNode::publish_trajectory_setpoint(const geometry_msgs::msg::PoseStamped& wp)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	const std::string& src = wp.header.frame_id;
	const std::string anchor = ned_anchor_frame();
	geometry_msgs::msg::Quaternion ori_nedworld;  // wp heading expressed in ned anchor (FLU body)
	if (!src.empty() && src != anchor) {
		try {
			geometry_msgs::msg::TransformStamped tf =
				tf_buffer_->lookupTransform(anchor, src, tf2::TimePointZero);
			geometry_msgs::msg::PoseStamped wp_ned;
			tf2::doTransform(wp, wp_ned, tf);  // rotates BOTH position and orientation
			msg.position[0] = (float)wp_ned.pose.position.x;
			msg.position[1] = (float)wp_ned.pose.position.y;
			msg.position[2] = (float)wp_ned.pose.position.z;
			ori_nedworld    = wp_ned.pose.orientation;
		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
				"[AUTO] setpoint TF failed (%s) — skipping", ex.what());
			return;
		}
	} else {
		// frame_id empty or already the anchor frame — use coordinates directly
		msg.position[0] = (float)wp.pose.position.x;
		msg.position[1] = (float)wp.pose.position.y;
		msg.position[2] = (float)wp.pose.position.z;
		ori_nedworld    = wp.pose.orientation;
	}

	msg.velocity[0] = msg.velocity[1] = msg.velocity[2]         = NaNf;
	msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = NaNf;
	msg.yaw = setpoint_yaw_ned(ori_nedworld);  // F1: track planner heading (NaN if none)
	trajectory_setpoint_pub_->publish(msg);
}

// Post-touchdown land-detect assist (2026-07-13): xy position hold from the cached
// landing setpoint (same TF path as publish_trajectory_setpoint), z position RELEASED,
// z velocity = +landing_detect_vz_ (NED down). See header note for the PX4 rationale.
void M4ControllerNode::publish_landing_detect_setpoint()
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	const geometry_msgs::msg::PoseStamped& wp = landing_setpoint_ps_;
	const std::string& src = wp.header.frame_id;
	const std::string anchor = ned_anchor_frame();
	geometry_msgs::msg::Quaternion ori_nedworld;
	if (!src.empty() && src != anchor) {
		try {
			geometry_msgs::msg::TransformStamped tf =
				tf_buffer_->lookupTransform(anchor, src, tf2::TimePointZero);
			geometry_msgs::msg::PoseStamped wp_ned;
			tf2::doTransform(wp, wp_ned, tf);
			msg.position[0] = (float)wp_ned.pose.position.x;
			msg.position[1] = (float)wp_ned.pose.position.y;
			ori_nedworld    = wp_ned.pose.orientation;
		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
				"[AUTO] landing-detect setpoint TF failed (%s) — skipping", ex.what());
			return;
		}
	} else {
		msg.position[0] = (float)wp.pose.position.x;
		msg.position[1] = (float)wp.pose.position.y;
		ori_nedworld    = wp.pose.orientation;
	}
	msg.position[2] = NaNf;                              // z position released
	msg.velocity[0] = msg.velocity[1] = NaNf;
	msg.velocity[2] = (float)landing_detect_vz_;         // NED: positive = down
	msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = NaNf;
	msg.yaw = setpoint_yaw_ned(ori_nedworld);
	trajectory_setpoint_pub_->publish(msg);
}

// Publish a full PVA setpoint (F2). Position + heading are converted to the PX4 NED frame
// exactly like publish_trajectory_setpoint (same TF path); velocity & acceleration are
// rotated into ned_world as feed-forward. Inputs are in sp.header.frame_id (odom ENU).
// NaN components pass through (PX4 leaves that axis uncontrolled).
void M4ControllerNode::publish_trajectory_setpoint_pva(const custom_msgs::msg::M4TrajectorySetpoint& sp)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	const std::string& src = sp.header.frame_id;
	const std::string anchor = ned_anchor_frame();
	geometry_msgs::msg::Quaternion ori_nedworld;
	geometry_msgs::msg::Vector3 vel_ned = sp.velocity;      // default: already in target frame
	geometry_msgs::msg::Vector3 acc_ned = sp.acceleration;
	if (!src.empty() && src != anchor) {
		try {
			geometry_msgs::msg::TransformStamped tf =
				tf_buffer_->lookupTransform(anchor, src, tf2::TimePointZero);
			// position + orientation: rotate both via a PoseStamped (same as waypoint)
			geometry_msgs::msg::PoseStamped wp, wp_ned;
			wp.header           = sp.header;
			wp.pose.position    = sp.position;
			wp.pose.orientation = sp.orientation;
			tf2::doTransform(wp, wp_ned, tf);
			msg.position[0] = (float)wp_ned.pose.position.x;
			msg.position[1] = (float)wp_ned.pose.position.y;
			msg.position[2] = (float)wp_ned.pose.position.z;
			ori_nedworld    = wp_ned.pose.orientation;
			// velocity / acceleration: rotation only (Vector3 ignores translation)
			geometry_msgs::msg::Vector3Stamped vin, vout;
			vin.header = sp.header;
			vin.vector = sp.velocity;     tf2::doTransform(vin, vout, tf); vel_ned = vout.vector;
			vin.vector = sp.acceleration; tf2::doTransform(vin, vout, tf); acc_ned = vout.vector;
		} catch (const tf2::TransformException& ex) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
				"[AUTO] pva setpoint TF failed (%s) — skipping", ex.what());
			return;
		}
	} else {
		msg.position[0] = (float)sp.position.x;
		msg.position[1] = (float)sp.position.y;
		msg.position[2] = (float)sp.position.z;
		ori_nedworld    = sp.orientation;
	}

	msg.velocity[0]     = (float)vel_ned.x;
	msg.velocity[1]     = (float)vel_ned.y;
	msg.velocity[2]     = (float)vel_ned.z;
	msg.acceleration[0] = (float)acc_ned.x;
	msg.acceleration[1] = (float)acc_ned.y;
	msg.acceleration[2] = (float)acc_ned.z;
	msg.yaw      = setpoint_yaw_ned(ori_nedworld);  // F1 helper
	// yawspeed: ENU up axis -> NED down axis (sign flip); NaN passes through
	msg.yawspeed = std::isnan(sp.yawspeed) ? NaNf : (float)(-sp.yawspeed);
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

		case OffboardState::TRACKING: {
			// F2: in pva mode, stream the full PVA setpoint while it is fresh; fall back to
			// the cached position setpoint (pose path) otherwise. The cached position is kept
			// current by the pva callback too, so the fallback is seamless.
			const bool pva_fresh = pva_input_ && has_pva_ &&
				(this->now() - last_pva_time_).seconds() < waypoint_timeout_s_;
			if (pva_fresh) {
				publish_offboard_control_mode(true, /*pva=*/true);
				publish_trajectory_setpoint_pva(latest_pva_);
			} else {
				publish_offboard_control_mode(true);
				publish_trajectory_setpoint(last_position_setpoint_);
			}
			return;
		}

		case OffboardState::DESCENDING:
			publish_offboard_control_mode(false);
			publish_descent_setpoint();
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				"[AUTO] DESCENDING — odom lost. Restore odom and resend mode_cmd=1 to resume.");
			return;

		case OffboardState::LANDING: {
			if (landing_disarm_wait_) {
				// Touchdown confirmed (our two-layer detection): switch to the
				// land-detect assist setpoint — xy position hold, z position released,
				// z VELOCITY = +landing_detect_vz_ (NED). PX4's land detector needs a
				// FINITE commanded descent >= 0.9*MPC_LAND_SPEED to declare ground
				// contact; pose-only setpoints (vz=NaN) keep it blind forever
				// (2026-07-13 ulog: 21s of "Disarming denied: not landed"). Harmless
				// on the ground — nothing actually moves. The stream never gaps.
				publish_offboard_control_mode(true, /*pva=*/true);
				publish_landing_detect_setpoint();
			} else {
				// Descent phase unchanged: position z decrement in the cached frame.
				landing_setpoint_ps_.pose.position.z -= (float)(landing_descent_mps_ * 0.1);
				publish_offboard_control_mode(true);
				publish_trajectory_setpoint(landing_setpoint_ps_);
			}
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

// Relocalization variant: identical to convert_odom_flu_to_ned() except the
// global NED anchor is ned_map instead of ned_world.
// TF chain: ned_map ← map ← camera_init ← body ← base_link ← ned_base_link
//   (map ← camera_init is scan_lock's dynamic lock; the position therefore
//    reflects the localized global pose and jumps on relock — PX4's EV fusion
//    absorbs it as a position reset. The velocity uses the SMOOTH msg twist
//    (/localization/odom, world frame) so the feed-forward stays clean.)
// Requires the static TF map→ned_map (180° about X, added in m4_relocalization.launch.py).
px4_msgs::msg::VehicleOdometry M4ControllerNode::convert_odom_flu_to_ned_relocal(
	const nav_msgs::msg::Odometry& ros_odom)
{
	px4_msgs::msg::VehicleOdometry px4_odom{};
	px4_odom.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
	px4_odom.timestamp_sample = px4_odom.timestamp;

	geometry_msgs::msg::TransformStamped tf_pose =
		tf_buffer_->lookupTransform("ned_map", "ned_base_link", tf2::TimePointZero);

	px4_odom.position[0] = (float)tf_pose.transform.translation.x;
	px4_odom.position[1] = (float)tf_pose.transform.translation.y;
	px4_odom.position[2] = (float)tf_pose.transform.translation.z;
	px4_odom.pose_frame  = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

	const auto& q = tf_pose.transform.rotation;
	px4_odom.q[0] = (float)q.w;
	px4_odom.q[1] = (float)q.x;
	px4_odom.q[2] = (float)q.y;
	px4_odom.q[3] = (float)q.z;

	// Linear velocity: world FLU → ned_map NED (smooth /localization/odom twist)
	geometry_msgs::msg::Vector3Stamped vel_in, vel_out;
	vel_in.header = ros_odom.header;   // frame_id = "world"
	vel_in.vector = ros_odom.twist.twist.linear;
	tf_buffer_->transform(vel_in, vel_out, "ned_map");

	px4_odom.velocity[0]    = (float)vel_out.vector.x;
	px4_odom.velocity[1]    = (float)vel_out.vector.y;
	px4_odom.velocity[2]    = (float)vel_out.vector.z;
	px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

	// Angular velocity: base_link FLU → ned_base_link FRD (body frame, anchor-agnostic)
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
	// NOTE (2026-07-10): the DISARM is now sent BEFORE this function, in the LANDING
	// touchdown branch; this finalizer runs only after PX4 confirmed the disarm
	// (layer-2 transactional landing). Never call it while PX4 is still ARMED.
	send_stabilized_mode_command();
	offboard_state_    = OffboardState::INACTIVE;
	landing_z_stopped_ = false;
	has_new_mode_cmd_ = true;
	latest_mode_cmd_  = pending_landing_mode_cmd_;
	// arm the post-landing disarm guard: until the A2G transform completes, any PX4
	// re-arm (level-triggered RC arm switch + residual throttle) is actively disarmed
	// again in loop() — re-takeoff must go through an explicit mode_cmd=1.
	post_landing_disarm_guard_ = true;
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
		post_landing_disarm_guard_ = false;  // explicit takeoff request lifts the guard
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
		landing_lowthr_count_     = 0;      // re-arm layer-1 debounce
		landing_disarm_wait_      = false;  // re-arm layer-2 transaction
		landing_z_hist_.clear();            // 3b: fresh differentiation window
		offboard_state_     = OffboardState::LANDING;
		RCLCPP_INFO(this->get_logger(),
			"[AUTO] Controlled landing initiated (target mode=%d after touchdown)", mode);
		return;
	}

	// Guard (2026-07-10 incident): a REPEAT ground mode_cmd while LANDING is already in
	// progress (e.g. the executor's A2G firing after a manual mode0 started the landing)
	// must NOT fall through to the deactivation branch below — that branch DISARMs
	// (denied mid-air) and then unconditionally switches STABILIZED + kills the offboard
	// heartbeat, leaving an armed aircraft with no position hold. Landing is idempotent:
	// refresh the queued post-touchdown target and keep descending.
	if (offboard_state_ == OffboardState::LANDING) {
		pending_landing_mode_cmd_ = cmd;
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"[AUTO] mode_cmd=%d while LANDING — landing already in progress (target queued)",
			mode);
		return;
	}
	// Same mid-air hazard for DESCENDING (odom lost, airborne velocity descent): ignore
	// ground commands until the descent completes / odom is restored.
	if (offboard_state_ == OffboardState::DESCENDING) {
		RCLCPP_WARN(this->get_logger(),
			"[AUTO] mode_cmd=%d ignored during DESCENDING (odom-lost descent in progress)",
			mode);
		return;
	}

	// Deactivate offboard if in the remaining aerial state (PRE_ARM — on the ground,
	// disarm is safe there)
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
		// Layer 2 — transactional DISARM. While waiting: the heartbeat keeps the
		// offboard stream alive with a FROZEN setpoint (no further z decrement), so a
		// PX4 denial ("Disarming denied, not landed") can never leave the aircraft in
		// armed STABILIZED with no position hold (2026-07-10 hardware incident).
		if (landing_disarm_wait_) {
			if (pixhawk_state.arming_state !=
			    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
				RCLCPP_INFO(this->get_logger(),
					"[AUTO] PX4 DISARM confirmed → STABILIZED + A2G");
				landing_disarm_wait_ = false;
				confirm_landing();
			} else if (this->now() > landing_disarm_deadline_) {
				RCLCPP_ERROR(this->get_logger(),
					"[AUTO] DISARM denied/timeout (PX4 still ARMED, likely mid-air) "
					"→ resuming controlled descent");
				landing_disarm_wait_  = false;
				landing_z_stopped_    = false;   // re-arm both detection layers
				landing_lowthr_count_ = 0;
			} else if ((this->now() - landing_disarm_last_send_).seconds() >= 0.5) {
				// 3a: PX4 'landed' may lag the first DISARM on a real touchdown —
				// re-send inside the window rather than one-shot + full timeout.
				// After landing_force_disarm_s_ of denials, escalate to FORCE (21196):
				// our own touchdown detection is the authority (see header note).
				const bool force = landing_force_disarm_s_ > 0.0 &&
					(this->now() - landing_disarm_wait_start_).seconds() >= landing_force_disarm_s_;
				if (force)
					RCLCPP_WARN(this->get_logger(),
						"[AUTO] normal DISARM window expired — escalating to FORCE disarm");
				send_arm_command(false, force);
				landing_disarm_last_send_ = this->now();
			}
			return;
		}
		// Touchdown requires BOTH conditions; otherwise keep the controlled descent:
		//   (1) vertical motion stopped: |z_vel| < landing_z_stop_thr_ held for
		//       landing_z_stop_dur_ (odom Z velocity — NOT ground height, so rugged
		//       terrain / planner ground-height error does not affect it), AND
		//   (2) PX4 reports has_low_throttle (position controller cut throttle because the
		//       vehicle is supported by the ground). PX4's `landed` is unreliable during an
		//       offboard position-controlled descent (it needs at_rest/ground_contact which
		//       don't trip), but has_low_throttle does. A normal hover does NOT set
		//       has_low_throttle, so it gates out the hover z-stop false-positive; z-stop
		//       gates out the descent phase (z_vel high) → together = on the ground.
		bool z_stopped = false;
		if (odom_healthy_) {
			// 3b: z velocity from POSITION differentiation over a ~0.5s window (odom
			// twist proved unreliable — zero or vibration-noisy depending on the LIO
			// source). Window too short right after LANDING entry -> not stopped yet.
			const rclcpp::Time tnow = this->now();
			landing_z_hist_.emplace_back(tnow, last_odom_.pose.pose.position.z);
			while (!landing_z_hist_.empty() &&
			       (tnow - landing_z_hist_.front().first).seconds() > 0.6)
				landing_z_hist_.pop_front();
			const double span = (tnow - landing_z_hist_.front().first).seconds();
			if (span >= 0.3) {
				const double z_vel = std::abs(
					(last_odom_.pose.pose.position.z - landing_z_hist_.front().second)
					/ span);
				if (z_vel < landing_z_stop_thr_) {
					if (!landing_z_stopped_) {
						landing_z_stopped_    = true;
						landing_z_stop_start_ = this->now();
					} else if ((this->now() - landing_z_stop_start_).seconds() >= landing_z_stop_dur_) {
						z_stopped = true;
					}
				} else {
					landing_z_stopped_ = false;
				}
			}
		}
		// Layer 1 — debounce has_low_throttle: it is an instantaneous PX4 flag with no
		// hysteresis, and it also trips MID-AIR when the unbounded landing setpoint
		// saturates the descent controller (thrust pushed into the bottom 10% of the
		// min→hover band). Count consecutive loop() frames (20Hz): only a sustained
		// low-throttle (default 30 frames ≈ 1.5s) passes.
		const bool lowthr_now = has_land_detected_ && last_land_detected_.has_low_throttle;
		landing_lowthr_count_ = lowthr_now ? landing_lowthr_count_ + 1 : 0;
		const bool low_throttle = landing_lowthr_count_ >= landing_lowthr_frames_;
		if (z_stopped && low_throttle) {
			RCLCPP_INFO(this->get_logger(),
				"[AUTO] Touchdown detected (z-stop + low_throttle x%d frames) "
				"→ DISARM sent, awaiting PX4 confirmation",
				landing_lowthr_count_);
			send_arm_command(false);
			landing_disarm_wait_       = true;
			landing_disarm_last_send_  = this->now();
			landing_disarm_wait_start_ = this->now();
			landing_disarm_deadline_   = this->now() +
				rclcpp::Duration::from_seconds(landing_disarm_timeout_s_);
		}
		return;
	}

	// ── TRACKING: update position setpoint cache for the heartbeat timer ──
	// F2 pva mode: the pva callback keeps last_position_setpoint_ current while PVA is
	// fresh; on staleness fall back to an odom hover target (mirrors the pose path below).
	if (pva_input_) {
		const double pva_age = has_pva_
			? (this->now() - last_pva_time_).seconds()
			: std::numeric_limits<double>::max();
		if (pva_age >= waypoint_timeout_s_ && odom_healthy_) {
			// Hover at the LAST COMMANDED waypoint, not at odom: last_position_setpoint_
			// already caches the last PVA position (see trajectory_setpoint_pva_callback),
			// so simply DON'T overwrite it. Holding the commanded target keeps the vehicle
			// converging to where the planner wanted it (e.g. finishing a climb) instead of
			// freezing mid-way at whatever pose the timeout caught it in. Odom fallback only
			// when no waypoint was ever received (nothing better to hold).
			if (!has_pva_) last_position_setpoint_ = odom_to_posestamped();
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				has_pva_ ? "[AUTO] trajectory_waypoint timeout — hovering at last waypoint"
				         : "[AUTO] No trajectory_waypoint received — hovering at odom");
		}
		return;
	}

	const double wp_age = has_waypoint_
		? (this->now() - last_waypoint_time_).seconds()
		: std::numeric_limits<double>::max();

	if (wp_age < waypoint_timeout_s_) {
		last_position_setpoint_ = latest_waypoint_;
	} else {
		if (odom_healthy_) {
			if (has_waypoint_) {
				// Hover at the LAST COMMANDED waypoint (keep converging to the planner's
				// last target, e.g. finish a climb) instead of freezing at odom mid-way.
				last_position_setpoint_ = latest_waypoint_;
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"[AUTO] Waypoint timeout (%.2fs > %.2fs) — hovering at last waypoint",
					wp_age, waypoint_timeout_s_);
			} else {
				// never received any waypoint: odom hover is the only sane target
				last_position_setpoint_ = odom_to_posestamped();
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"[AUTO] No waypoint received — hovering at odom position");
			}
		}
		// !odom_healthy_ → loop() odom tick triggers TRACKING → DESCENDING
	}
}
