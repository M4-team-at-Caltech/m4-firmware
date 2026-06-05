#include <string>
#include <vector>
#include <atomic>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "src/RCcontroller.h"
#include "M4Base.h"

// #define DRY_RUN
using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

static constexpr float NaNf = std::numeric_limits<float>::quiet_NaN();

// ─────────────────────────────────────────────────────────────────────────────
// Transition handler structs (unchanged)
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
	M4ControllerNode(std::vector<int> active_joint_ids, std::vector<int> active_wheel_ids)
		: Node("m4_controller_node"),
		  m4_configuration(M4Configurations::INIT),
		  m4(nullptr),
		  rc_controller(this)
	{
		// Initialize M4 base class
		#ifndef DRY_RUN
			m4 = new M4Base(active_joint_ids, active_wheel_ids, false);
		#endif

		// Define mode transition handlers
		drive_mode_transition_handler = {
			{ DriveSwitchState::STAND,   M4Configurations::STAND,   "Move to stand",   &M4Base::stand   },
			{ DriveSwitchState::CROUCH1, M4Configurations::CROUCH1, "Move to crouch1", &M4Base::crouch1 },
			{ DriveSwitchState::CROUCH2, M4Configurations::CROUCH2, "Move to crouch2", &M4Base::crouch2 }
		};
		uav_mode_transition_handler = {
			{ UAVSwitchState::UAV_DISABLE,   M4Configurations::UAV, "Move to stand", &M4Base::stand },
			{ UAVSwitchState::UAV_TRANSFORM, M4Configurations::UAV, "Move to uav",   &M4Base::uav   }
		};

		// ── Timers ──
		// 50 ms main control loop
		timer_ = this->create_wall_timer(50ms, std::bind(&M4ControllerNode::loop, this));

		// 10 Hz dedicated offboard heartbeat — runs independently of loop()
		// guarantees >2 Hz even if loop() is delayed by init or early-returns
		offboard_timer_ = this->create_wall_timer(
			100ms, std::bind(&M4ControllerNode::offboard_heartbeat_callback, this));

		// ── PX4 QoS ──
		rclcpp::QoS px4_qos(10);
		px4_qos.best_effort();
		px4_qos.transient_local();

		// ── Subscribers ──
		state_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
			"/fmu/out/vehicle_status", px4_qos,
			std::bind(&M4ControllerNode::state_callback, this, _1));

		vehicle_land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
			"/fmu/out/vehicle_land_detected", px4_qos,
			std::bind(&M4ControllerNode::vehicle_land_detected_callback, this, _1));

		// AUTO mode inputs
		auto_mode_cmd_sub_ = this->create_subscription<custom_msgs::msg::M4ModeCmd>(
			"/auto/mode_cmd", 10,
			std::bind(&M4ControllerNode::mode_cmd_callback, this, _1));

		auto_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"/auto/waypoint", 10,
			std::bind(&M4ControllerNode::waypoint_callback, this, _1));

		auto_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
			"/auto/cmd_vel", 10,
			std::bind(&M4ControllerNode::cmd_vel_callback, this, _1));

		// Visual odometry input (source configured in robot_config.yaml)
		load_auto_config();
		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			odom_topic_, 10,
			std::bind(&M4ControllerNode::odom_callback, this, _1));

		// ── Publishers ──
		vehicle_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_control_mode_pub_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);

		trajectory_setpoint_pub_ =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

		// Odometry output topic depends on source type
		const std::string odom_fmu_topic = (odom_source_ == "mocap")
			? "/fmu/in/vehicle_mocap_odometry"
			: "/fmu/in/vehicle_visual_odometry";
		vehicle_odom_pub_ =
			this->create_publisher<px4_msgs::msg::VehicleOdometry>(odom_fmu_topic, 10);

		mode_status_pub_ =
			this->create_publisher<custom_msgs::msg::M4ModeStatus>("/auto/mode_status", 10);

		// TF2 buffer + listener for FLU→NED odometry conversion
		tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

		RCLCPP_INFO(this->get_logger(),
			"[m4_controller_node] AUTO mode ready. odom_source=%s topic=%s fmu_out=%s descent=%.2fm/s",
			odom_source_.c_str(), odom_topic_.c_str(), odom_fmu_topic.c_str(), descent_rate_mps_);
		cout << "rc_controller_init" << endl;
	}

	~M4ControllerNode()
	{
		#ifndef DRY_EXECUTION_MODE
			if (m4 != nullptr) {
				m4->sit();
				while (m4->get_current_configuration() != M4Configurations::SIT) {
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
				}
				delete m4;
				m4 = nullptr;
			}
		#endif
		RCLCPP_WARN(this->get_logger(), "M4 Controller Node Shutting Down!");
	}

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
	double      descent_rate_mps_{0.3};   // downward speed (m/s) when odom is lost (DESCENDING)
	double      arm_imu_wait_s_{3.0};     // wait after G2A transform before ARM (IMU settle)
	double      waypoint_timeout_s_{2.0}; // aerial waypoint staleness threshold; hover at odom if exceeded
	double      landing_descent_mps_{0.3};  // controlled descent rate during LANDING (m/s)
	double      landing_z_stop_thr_{0.05};  // odom Z velocity threshold to detect touchdown (m/s)
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
	// Decoupled from loop() — drives the dedicated 10Hz heartbeat timer.
	//   INACTIVE   : heartbeat not sent, PX4 not in offboard
	//   PRE_ARM    : IMU settle wait + ARM retry window — heartbeat sent (position=true)
	//   TRACKING   : position control, publishes OffboardControlMode(position=true) + last setpoint
	//   DESCENDING : velocity control, publishes OffboardControlMode(velocity=true) + descent cmd
	//                entered when odom is lost mid-flight; NOT auto-cleared on odom recovery
	//                (requires explicit mode_cmd=1 to resume TRACKING)
	//   LANDING    : position control with decreasing Z; entered on A2G cmd while TRACKING
	//                confirms touchdown via vehicle_land_detected or odom Z stop
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

	// ─────────────────────────────────────────────────────────────────────────
	// Config loader
	// ─────────────────────────────────────────────────────────────────────────
	void load_auto_config()
	{
		try {
			std::string path = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
			YAML::Node cfg  = YAML::LoadFile(path);
			if (cfg["auto_mode"]) {
				odom_source_       = cfg["auto_mode"]["odom_source"].as<std::string>(odom_source_);
				odom_topic_        = cfg["auto_mode"]["odom_topic"].as<std::string>(odom_topic_);
				odom_timeout_s_    = cfg["auto_mode"]["odom_timeout_s"].as<double>(odom_timeout_s_);
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

	// ─────────────────────────────────────────────────────mode_cmd_callback────────────────────
	// Callbacks
	// ─────────────────────────────────────────────────────────────────────────
	void state_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
	{
		pixhawk_state = *msg;
	}

	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		last_odom_      = *msg;
		last_odom_time_ = this->now();
		odom_healthy_   = true;

		// Forward to PX4 via TF2-based FLU→NED conversion
		try {
			vehicle_odom_pub_->publish(convert_odom_flu_to_ned(*msg));
		} catch (const tf2::TransformException& e) {
			RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
				"[odom] TF ned_base_link→ned_world not ready: %s", e.what());
		}
	}

	void vehicle_land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
	{
		last_land_detected_ = *msg;
		has_land_detected_  = true;
	}

	void mode_cmd_callback(const custom_msgs::msg::M4ModeCmd::SharedPtr msg)
	{
		latest_mode_cmd_  = *msg;
		has_new_mode_cmd_ = true;
	}

	void waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		latest_waypoint_        = *msg;
		last_waypoint_time_     = this->now();
		has_waypoint_           = true;
		last_position_setpoint_ = *msg;  // keep last valid for heartbeat timer
	}

	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		latest_cmd_vel_    = *msg;
		last_cmd_vel_time_ = this->now();
		has_cmd_vel_       = true;
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Init
	// ─────────────────────────────────────────────────────────────────────────
	void init_controller()
	{
		#ifndef DRY_RUN
			m4->sit();
			std::cout << "Sit complete!\n";
			while (m4->get_current_configuration() != M4Configurations::SIT)
				std::this_thread::sleep_for(std::chrono::milliseconds(100));

			m4->stand();
			while (m4->get_current_configuration() != M4Configurations::STAND)
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
			m4_configuration = m4->get_current_configuration();
		#else
			m4_configuration = M4Configurations::STAND;
		#endif

		RCLCPP_WARN(this->get_logger(), "M4 Controller Node Initialized!");
	}

	// ─────────────────────────────────────────────────────────────────────────
	// PX4 command helpers
	// ─────────────────────────────────────────────────────────────────────────
	void send_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f)
	{
		px4_msgs::msg::VehicleCommand cmd{};
		cmd.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
		cmd.command          = command;
		cmd.param1           = param1;
		cmd.param2           = param2;
		cmd.target_system    = 1;
		cmd.target_component = 1;
		cmd.source_system    = 1;
		cmd.source_component = 1;
		cmd.from_external    = true;
		vehicle_command_publisher_->publish(cmd);
	}

	void send_arm_command(bool arm)
	{
		send_vehicle_command(
			px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
			arm ? 1.0f : 0.0f);
		RCLCPP_INFO(this->get_logger(), "[AUTO] UAV %s command sent to PX4",
			arm ? "ARM" : "DISARM");
	}

	void send_offboard_mode_command()
	{
		// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, custom_mode=6 (offboard)
		send_vehicle_command(
			px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
	}

	// Switch PX4 back to STABILIZED mode (param2=7) so manual RC arming works again.
	// Must be called whenever offboard_state_ transitions from non-INACTIVE to INACTIVE,
	// otherwise PX4 stays in OFFBOARD flight mode and QGC requires manual mode switch.
	void send_stabilized_mode_command()
	{
		send_vehicle_command(
			px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 7.0f);
		RCLCPP_INFO(this->get_logger(), "[AUTO] PX4 flight mode → STABILIZED");
	}

	// DISARM + stabilized mode + re-queue pending ground mode_cmd to trigger A2G transform.
	// Called when LANDING touchdown is confirmed (vehicle_land_detected or odom Z stop).
	void confirm_landing()
	{
		send_arm_command(false);
		send_stabilized_mode_command();
		offboard_state_    = OffboardState::INACTIVE;
		landing_z_stopped_ = false;
		// Re-trigger the stored ground mode cmd; next loop() tick will call process_mode_cmd
		// which handles A2G transform and drives to the correct ground configuration.
		has_new_mode_cmd_ = true;
		latest_mode_cmd_  = pending_landing_mode_cmd_;
	}

	// Publish OffboardControlMode — position_mode=true → position ctrl,  false → velocity ctrl
	void publish_offboard_control_mode(bool position_mode)
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

	// Publish position setpoint (world/FLU PoseStamped → NED TrajectorySetpoint)
	void publish_trajectory_setpoint(const geometry_msgs::msg::PoseStamped& wp)
	{
		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.timestamp   = this->get_clock()->now().nanoseconds() / 1000;
		msg.position[0] =  (float)wp.pose.position.x;  // North = FLU Forward
		msg.position[1] = -(float)wp.pose.position.y;  // East  = -FLU Left
		msg.position[2] = -(float)wp.pose.position.z;  // Down  = -FLU Up
		msg.velocity[0] = msg.velocity[1] = msg.velocity[2]         = NaNf;
		msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = NaNf;
		msg.yaw         = NaNf;
		trajectory_setpoint_pub_->publish(msg);
	}

	// Publish controlled descent setpoint (velocity mode, no horizontal movement)
	void publish_descent_setpoint()
	{
		px4_msgs::msg::TrajectorySetpoint msg{};
		msg.timestamp     = this->get_clock()->now().nanoseconds() / 1000;
		msg.position[0]   = msg.position[1]   = msg.position[2]   = NaNf;
		msg.velocity[0]   = 0.0f;  // North: no drift
		msg.velocity[1]   = 0.0f;  // East:  no drift
		msg.velocity[2]   = (float)descent_rate_mps_;  // Down: positive = descend (NED)
		msg.acceleration[0] = msg.acceleration[1] = msg.acceleration[2] = NaNf;
		msg.yaw           = NaNf;
		trajectory_setpoint_pub_->publish(msg);
	}

	// ─────────────────────────────────────────────────────────────────────────
	// 10 Hz offboard heartbeat timer callback
	// Publishes OffboardControlMode + setpoint according to offboard_state_.
	// Runs independently of loop() to guarantee >2 Hz even when loop() is
	// delayed by init, DISARM early-return, or other blocking work.
	// ─────────────────────────────────────────────────────────────────────────
	void offboard_heartbeat_callback()
	{
		switch (offboard_state_) {
			case OffboardState::INACTIVE:
				return;  // Not in offboard — send nothing

			case OffboardState::PRE_ARM:
				// Warm up the offboard stream during IMU settle wait.
				// PX4 requires OffboardControlMode for a stable period before ARM.
				publish_offboard_control_mode(true);   // position=true
				return;  // No setpoint yet — ARM hasn't been sent

			case OffboardState::TRACKING:
				publish_offboard_control_mode(true);   // position=true
				publish_trajectory_setpoint(last_position_setpoint_);
				return;

			case OffboardState::DESCENDING:
				publish_offboard_control_mode(false);  // velocity=true
				publish_descent_setpoint();
				RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
					"[AUTO] DESCENDING — odom lost. Restore odom and resend mode_cmd=1 to resume.");
				return;

			case OffboardState::LANDING: {
				// Decrease Z setpoint each tick (10 Hz → 0.1 s per tick)
				landing_setpoint_z_ -= (float)(landing_descent_mps_ * 0.1);
				geometry_msgs::msg::PoseStamped landing_ps;
				landing_ps.pose.position.x = landing_setpoint_x_;
				landing_ps.pose.position.y = landing_setpoint_y_;
				landing_ps.pose.position.z = landing_setpoint_z_;
				publish_offboard_control_mode(true);  // position mode
				publish_trajectory_setpoint(landing_ps);
				return;
			}
		}
	}

	// Map M4Configurations → int32 (matches M4ModeCmd.mode values)
	static int32_t config_to_int(M4Configurations cfg)
	{
		switch (cfg) {
			case M4Configurations::STAND:   return 0;
			case M4Configurations::UAV:     return 1;
			case M4Configurations::CROUCH1: return 2;
			case M4Configurations::CROUCH2: return 4;
			case M4Configurations::SIT:     return -2;
			default:                        return -1;  // INIT / UNKNOWN
		}
	}

	// Map OffboardState → int32
	static int32_t offboard_state_to_int(OffboardState s)
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

	// Publish current controller state to /auto/mode_status (50 Hz, called from loop())
	void publish_mode_status()
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

	// Build a PoseStamped hover setpoint from the latest odom (ENU frame).
	// Used to initialise last_position_setpoint_ at ARM time and during waypoint timeout.
	geometry_msgs::msg::PoseStamped odom_to_posestamped() const
	{
		geometry_msgs::msg::PoseStamped ps;
		ps.header.stamp    = last_odom_.header.stamp;
		ps.header.frame_id = "map";
		ps.pose            = last_odom_.pose.pose;
		// Ensure valid unit quaternion if odom orientation is degenerate
		const auto& q = ps.pose.orientation;
		if (std::abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1.0) > 0.01) {
			ps.pose.orientation = geometry_msgs::msg::Quaternion{};
			ps.pose.orientation.w = 1.0;
		}
		return ps;
	}

	// ─────────────────────────────────────────────────────────────────────────
	// Odometry conversion: FLU (base_link in world) → NED (ned_base_link in ned_world)
	// ─────────────────────────────────────────────────────────────────────────
	// TF chain: ned_world ← world ← camera_init ← body ← base_link ← ned_base_link
	// Requires static TFs: world→ned_world and base_link→ned_base_link (both 180° around X).
	// Throws tf2::TransformException if any transform in the chain is not yet available.
	px4_msgs::msg::VehicleOdometry convert_odom_flu_to_ned(
		const nav_msgs::msg::Odometry& ros_odom)
	{
		px4_msgs::msg::VehicleOdometry px4_odom{};
		px4_odom.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
		px4_odom.timestamp_sample = px4_odom.timestamp;

		// ── Pose: ned_base_link (FRD body) in ned_world (NED world) ───────────
		// lookupTransform resolves the full TF chain automatically.
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

		// ── Linear velocity: world FLU → ned_world NED ────────────────────────
		// base_world_odom.py outputs linear velocity rotated to world frame (FLU).
		geometry_msgs::msg::Vector3Stamped vel_in, vel_out;
		vel_in.header = ros_odom.header;   // frame_id = "world" ✓
		vel_in.vector = ros_odom.twist.twist.linear;
		tf_buffer_->transform(vel_in, vel_out, "ned_world");

		px4_odom.velocity[0]    = (float)vel_out.vector.x;
		px4_odom.velocity[1]    = (float)vel_out.vector.y;
		px4_odom.velocity[2]    = (float)vel_out.vector.z;
		px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

		// ── Angular velocity: base_link FLU → ned_base_link FRD ──────────────
		// Angular velocity from FastLIO is in body frame; child_frame_id = "base_link".
		geometry_msgs::msg::Vector3Stamped ang_in, ang_out;
		ang_in.header.frame_id = ros_odom.child_frame_id;  // "base_link"
		ang_in.header.stamp    = ros_odom.header.stamp;
		ang_in.vector          = ros_odom.twist.twist.angular;
		tf_buffer_->transform(ang_in, ang_out, "ned_base_link");

		px4_odom.angular_velocity[0] = (float)ang_out.vector.x;
		px4_odom.angular_velocity[1] = (float)ang_out.vector.y;
		px4_odom.angular_velocity[2] = (float)ang_out.vector.z;

		// Variances: NaN = unknown
		for (int i = 0; i < 3; i++) {
			px4_odom.position_variance[i]    = NaNf;
			px4_odom.orientation_variance[i] = NaNf;
			px4_odom.velocity_variance[i]    = NaNf;
		}
		px4_odom.quality = 1;
		return px4_odom;
	}

	// ─────────────────────────────────────────────────────────────────────────
	// AUTO mode — process incoming M4ModeCmd
	//
	//  mode 0 = STAND   → ground
	//  mode 1 = UAV     → aerial (arms PX4 if odom healthy)
	//  mode 2 = CROUCH1 → ground
	//  mode 4 = CROUCH2 → ground
	// ─────────────────────────────────────────────────────────────────────────
	void process_mode_cmd(const custom_msgs::msg::M4ModeCmd& cmd)
	{
		const int mode = cmd.mode;

		// ── mode 1: UAV (aerial) ──────────────────────────────────────────
		if (mode == 1) {
			if (m4_configuration != M4Configurations::UAV) {
				RCLCPP_WARN(this->get_logger(), "[AUTO] Move to uav");
				#ifndef DRY_RUN
					m4->drive(0, 0);
					m4->uav();
					m4_configuration = m4->get_current_configuration();
				#else
					m4_configuration = M4Configurations::UAV;
				#endif
			}
			auto_sub_state_    = AutoSubState::AERIAL;
			offboard_state_    = OffboardState::PRE_ARM;  // start heartbeat immediately
			arm_pending_       = true;
			arm_pending_since_ = this->now();
			send_offboard_mode_command();  // request OFFBOARD mode early so stream warms up
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
			landing_setpoint_x_ = (float)last_position_setpoint_.pose.position.x;
			landing_setpoint_y_ = (float)last_position_setpoint_.pose.position.y;
			landing_setpoint_z_ = (float)last_position_setpoint_.pose.position.z;
			landing_z_stopped_  = false;
			offboard_state_     = OffboardState::LANDING;
			RCLCPP_INFO(this->get_logger(),
				"[AUTO] Controlled landing initiated (target mode=%d after touchdown)", mode);
			return;
		}

		// Deactivate offboard if we were in other aerial states (PRE_ARM / DESCENDING)
		if (offboard_state_ != OffboardState::INACTIVE) {
			send_arm_command(false);
			send_stabilized_mode_command();  // exit OFFBOARD so manual RC arming works
			offboard_state_ = OffboardState::INACTIVE;
		}

		// If coming from UAV config, trigger A2G transform first
		if (m4_configuration == M4Configurations::UAV) {
			RCLCPP_WARN(this->get_logger(), "[AUTO] A2G transform — move to stand");
			#ifndef DRY_RUN
				m4->drive(0, 0);
				m4->stand();
				m4_configuration = m4->get_current_configuration();
			#else
				m4_configuration = M4Configurations::STAND;
			#endif
		}

		// Drive target configuration via handler table
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
					#ifndef DRY_RUN
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

	// ─────────────────────────────────────────────────────────────────────────
	// AUTO mode — ground execution (cmd_vel driven)
	// ─────────────────────────────────────────────────────────────────────────
	void execute_auto_ground()
	{
		const double cmd_vel_age = (this->now() - last_cmd_vel_time_).seconds();
		const bool   cmd_vel_ok  = has_cmd_vel_ && (cmd_vel_age < odom_timeout_s_);

		#ifndef DRY_RUN
			if (cmd_vel_ok) {
				// angular.z sign is inverted relative to M4Base differential drive convention
				m4->drive(latest_cmd_vel_.linear.x, -latest_cmd_vel_.angular.z);
			} else {
				m4->drive(0.0f, 0.0f);
				if (has_cmd_vel_)
					RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
						"[AUTO] cmd_vel timeout (%.2fs) — holding zero", cmd_vel_age);
			}
		#endif
	}

	// ─────────────────────────────────────────────────────────────────────────
	// AUTO mode — aerial execution
	// Note: heartbeat + setpoint publishing is handled entirely by offboard_timer_.
	// This function only manages state transitions and updates the setpoint cache.
	// ─────────────────────────────────────────────────────────────────────────
	void execute_auto_aerial()
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

		// ── LANDING: check touchdown sensors ─────────────────────────────────
		if (offboard_state_ == OffboardState::LANDING) {
			// Primary: PX4 native land detection
			if (has_land_detected_ && last_land_detected_.landed) {
				RCLCPP_INFO(this->get_logger(),
					"[AUTO] Touchdown confirmed (vehicle_land_detected) → DISARM + A2G");
				confirm_landing();
				return;
			}
			// Fallback: odom Z velocity stopped for landing_z_stop_dur_ seconds
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

	// ─────────────────────────────────────────────────────────────────────────
	// Main control loop (50 ms)
	// ─────────────────────────────────────────────────────────────────────────
	void loop()
	{
		if (m4_configuration == M4Configurations::INIT)
			init_controller();

		if (!rc_controller.get_rc_state_available())
			return;

		rc_state = rc_controller.get_rc_state();
		// ────────────────────────────────────────────────────────────────────
		// [①] DISARM — highest priority, overrides everything including AUTO
		// ────────────────────────────────────────────────────────────────────
		if (rc_state.arm_switch_state == ArmSwitchState::DISARM) {
			#ifndef DRY_RUN
				m4->drive(0, 0);
			#endif
			if (offboard_state_ != OffboardState::INACTIVE) {
				send_arm_command(false);
				send_stabilized_mode_command();  // exit OFFBOARD so manual RC arming works
				offboard_state_ = OffboardState::INACTIVE;
			}
			arm_pending_       = false;
			arm_retry_active_  = false;
			landing_z_stopped_ = false;
			auto_mode_active_  = false;
			auto_sub_state_    = AutoSubState::IDLE;
			// Reset prev_auto_switch_ so that re-arming with AUTO switch
			// still in AUTO position correctly re-triggers AUTO mode activation.
			prev_auto_switch_ = AutoSwitchState::MANUAL;
			publish_mode_status();
			return;
		}

		// Update current configuration from hardware
		#ifndef DRY_RUN
			m4_configuration = m4->get_current_configuration();
		#endif

		// ────────────────────────────────────────────────────────────────────
		// [②] AUTO switch transition detection
		// ────────────────────────────────────────────────────────────────────
		const bool just_entered_auto =
			(prev_auto_switch_ == AutoSwitchState::MANUAL &&
			 rc_state.auto_switch_state == AutoSwitchState::AUTO);

		if (just_entered_auto) {
			if (rc_state.arm_switch_state == ArmSwitchState::AERIAL_ARM) {
				RCLCPP_WARN(this->get_logger(),
					"[AUTO] Mode switch REJECTED: currently in AERIAL_ARM. "
					"Land and disarm before enabling AUTO.");
			} else {
				auto_mode_active_ = true;
				auto_sub_state_   = AutoSubState::IDLE;
				offboard_state_   = OffboardState::INACTIVE;
				RCLCPP_INFO(this->get_logger(), "[AUTO] AUTO mode activated");
			}
		}

		// Switching back to MANUAL always allowed
		if (rc_state.auto_switch_state == AutoSwitchState::MANUAL) {
			if (auto_mode_active_) {
				if (offboard_state_ != OffboardState::INACTIVE) {
					send_arm_command(false);
					send_stabilized_mode_command();  // exit OFFBOARD so manual RC arming works
					offboard_state_ = OffboardState::INACTIVE;
				}
				arm_pending_       = false;
				arm_retry_active_  = false;
				landing_z_stopped_ = false;
				auto_mode_active_  = false;
				auto_sub_state_    = AutoSubState::IDLE;
				RCLCPP_INFO(this->get_logger(), "[AUTO] AUTO mode deactivated — returning to MANUAL");
			}
		}

		prev_auto_switch_ = rc_state.auto_switch_state;

		// ── Odom health tick ──
		if (odom_healthy_ &&
		    (this->now() - last_odom_time_).seconds() > odom_timeout_s_) {
			odom_healthy_ = false;
			RCLCPP_ERROR(this->get_logger(),
				"[AUTO] Odometry timeout! Source: %s topic: %s",
				odom_source_.c_str(), odom_topic_.c_str());
			// TRACKING / LANDING → DESCENDING transition: odom lost mid-flight
			if (offboard_state_ == OffboardState::TRACKING ||
			    offboard_state_ == OffboardState::LANDING) {
				offboard_state_    = OffboardState::DESCENDING;
				landing_z_stopped_ = false;
				RCLCPP_ERROR(this->get_logger(),
					"[AUTO] Odom lost — switching to DESCENDING (%.2f m/s). "
					"Restore odom and resend mode_cmd=1 to resume.",
					descent_rate_mps_);
			}
		}

		// ────────────────────────────────────────────────────────────────────
		// [③] AUTO mode
		// ────────────────────────────────────────────────────────────────────
		if (auto_mode_active_) {
			if (has_new_mode_cmd_) {
				has_new_mode_cmd_ = false;
				process_mode_cmd(latest_mode_cmd_);
			}
			switch (auto_sub_state_) {
				case AutoSubState::GROUND:  execute_auto_ground(); break;
				case AutoSubState::AERIAL:  execute_auto_aerial(); break;
				case AutoSubState::IDLE:
				default: break;
			}
			publish_mode_status();
			return;  // Do not fall through to MANUAL logic
		}

		// ────────────────────────────────────────────────────────────────────
		// [④] MANUAL mode (original logic, unchanged)
		// ────────────────────────────────────────────────────────────────────
		if (rc_state.arm_switch_state == ArmSwitchState::GROUND_ARM) {
			if (rc_state.uav_switch_state == UAVSwitchState::UAV_DISABLE)
			{// Drive Mode
				for (const auto& handler : drive_mode_transition_handler)
				{
					if (rc_state.drive_switch_state == handler.state &&
					    m4_configuration != handler.configuration)
					{
						RCLCPP_WARN(this->get_logger(), "%s", handler.message);
						#ifndef DRY_RUN
							(m4->*handler.action)();
							m4_configuration = m4->get_current_configuration();
						#else
							m4_configuration = handler.configuration;
						#endif
						break;
					}
				}
				#ifndef DRY_RUN
					m4->drive(rc_state.velocity_joystick_state.x_velocity,
					          rc_state.velocity_joystick_state.y_velocity);
				#endif
			}
			else if (rc_state.uav_switch_state == UAVSwitchState::UAV_TRANSFORM)
			{// UAV Mode
				for (const auto& handler : uav_mode_transition_handler)
				{
					if (rc_state.uav_switch_state == handler.state &&
					    m4_configuration != handler.configuration)
					{
						RCLCPP_WARN(this->get_logger(), "%s", handler.message);
						#ifndef DRY_RUN
							m4->drive(0, 0);
							(m4->*handler.action)();
							m4_configuration = m4->get_current_configuration();
						#else
							m4_configuration = handler.configuration;
						#endif
						break;
					}
				}
			}
		}
		else if (rc_state.arm_switch_state == ArmSwitchState::AERIAL_ARM) {
			// If not in UAV configuration, override arming. IMPORTANT SAFETY CHECK!
			if (m4_configuration != M4Configurations::UAV) {
				RCLCPP_ERROR(this->get_logger(), "Not in UAV CONFIGURATION. Disarming!");
				send_arm_command(false);
				RCLCPP_INFO(this->get_logger(), "Disarm command sent to Pixhawk");
			}
		}

		// Publish status at end of every loop() tick (50 Hz)
		publish_mode_status();
	}
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	std::vector<int> active_joint_ids = {1, 2, 3, 4, 5, 6, 7, 8};
	std::vector<int> active_wheel_ids = {9, 10, 11, 12};

	auto node = std::make_shared<M4ControllerNode>(active_joint_ids, active_wheel_ids);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
