#include "m4_controller_node.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────
M4ControllerNode::M4ControllerNode(
	std::vector<int> active_joint_ids,
	std::vector<int> active_wheel_ids)
	: Node("m4_controller_node"),
	  m4_configuration(M4Configurations::INIT),
	  m4(nullptr),
	  rc_controller(this)
{
	#if defined(SIMULATION)
		m4 = new M4Base(active_joint_ids, active_wheel_ids, false, this);
	#elif !defined(DRY_EXECUTION_MODE)
		m4 = new M4Base(active_joint_ids, active_wheel_ids, false);
	#endif

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
	timer_ = this->create_wall_timer(50ms, std::bind(&M4ControllerNode::loop, this));
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

	auto_mode_cmd_sub_ = this->create_subscription<custom_msgs::msg::M4ModeCmd>(
		"/auto/mode_cmd", 10,
		std::bind(&M4ControllerNode::mode_cmd_callback, this, _1));

	auto_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
		"/auto/waypoint", 10,
		std::bind(&M4ControllerNode::waypoint_callback, this, _1));

	auto_trajectory_sub_ = this->create_subscription<custom_msgs::msg::M4TrajectorySetpoint>(
		"/auto/trajectory_waypoint", 10,
		std::bind(&M4ControllerNode::trajectory_setpoint_pva_callback, this, _1));

	auto_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		"/auto/cmd_vel", 10,
		std::bind(&M4ControllerNode::cmd_vel_callback, this, _1));

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

	const std::string odom_fmu_topic = (odom_source_ == "mocap")
		? "/fmu/in/vehicle_mocap_odometry"
		: "/fmu/in/vehicle_visual_odometry";
	vehicle_odom_pub_ =
		this->create_publisher<px4_msgs::msg::VehicleOdometry>(odom_fmu_topic, 10);

	mode_status_pub_ =
		this->create_publisher<custom_msgs::msg::M4ModeStatus>("/auto/mode_status", 10);

	tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	RCLCPP_INFO(this->get_logger(),
		"[m4_controller_node] AUTO mode ready. odom_source=%s topic=%s fmu_out=%s descent=%.2fm/s",
		odom_source_.c_str(), odom_topic_.c_str(), odom_fmu_topic.c_str(), descent_rate_mps_);
	cout << "rc_controller_init" << endl;
}

M4ControllerNode::~M4ControllerNode()
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

// ─────────────────────────────────────────────────────────────────────────────
// Init
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::init_controller()
{
	#ifndef DRY_EXECUTION_MODE
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

// ─────────────────────────────────────────────────────────────────────────────
// PX4 command helpers
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::send_vehicle_command(uint16_t command, float param1, float param2)
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

void M4ControllerNode::send_arm_command(bool arm, bool force)
{
	send_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		arm ? 1.0f : 0.0f,
		force ? 21196.0f : 0.0f);   // MAVLink force code: bypass the landed check
	RCLCPP_INFO(this->get_logger(), "[AUTO] UAV %s%s command sent to PX4",
		arm ? "ARM" : "DISARM", force ? " (FORCE)" : "");
}

void M4ControllerNode::send_offboard_mode_command()
{
	send_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

// Switch PX4 back to STABILIZED mode so manual RC arming works again after offboard exits.
void M4ControllerNode::send_stabilized_mode_command()
{
	send_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 7.0f);
	RCLCPP_INFO(this->get_logger(), "[AUTO] PX4 flight mode → STABILIZED");
}

// ─────────────────────────────────────────────────────────────────────────────
// Callbacks
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::state_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
	pixhawk_state = *msg;
}

void M4ControllerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	last_odom_      = *msg;
	last_odom_time_ = this->now();
	odom_healthy_   = true;

	try {
		vehicle_odom_pub_->publish(relocal_mode_ ? convert_odom_flu_to_ned_relocal(*msg)
		                                          : convert_odom_flu_to_ned(*msg));
	} catch (const tf2::TransformException& e) {
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
			"[odom] TF ned_base_link→%s not ready: %s",
			relocal_mode_ ? "ned_map" : "ned_world", e.what());
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// Main control loop (50 ms)
// ─────────────────────────────────────────────────────────────────────────────
void M4ControllerNode::loop()
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
		#ifndef DRY_EXECUTION_MODE
			m4->drive(0, 0);
		#endif
		if (offboard_state_ != OffboardState::INACTIVE) {
			send_arm_command(false);
			send_stabilized_mode_command();
			offboard_state_ = OffboardState::INACTIVE;
		}
		arm_pending_       = false;
		arm_retry_active_  = false;
		arm_failed_        = false;
		landing_z_stopped_ = false;
		auto_mode_active_  = false;
		auto_sub_state_    = AutoSubState::IDLE;
		has_new_mode_cmd_  = false;  // discard any pending A2G re-queue
		post_landing_disarm_guard_ = false;  // explicit kill switch overrides the guard
		// Reset so re-arming with AUTO switch still in AUTO re-triggers activation.
		prev_auto_switch_ = AutoSwitchState::MANUAL;
		publish_mode_status();
		return;
	}

	#ifndef DRY_EXECUTION_MODE
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

	if (rc_state.auto_switch_state == AutoSwitchState::MANUAL) {
		if (auto_mode_active_) {
			if (offboard_state_ != OffboardState::INACTIVE) {
				const bool airborne =
					m4_configuration == M4Configurations::UAV &&
					pixhawk_state.arming_state ==
						px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
				if (airborne) {
					// In-air handover (see auto_mode_DevNote 十七): keep ARMED, hand the
					// vehicle to the pilot in STABILIZED (the trained takeover procedure).
					// The old unconditional DISARM only "worked" because PX4 rejects
					// in-flight disarm without the force param — one parameter away from
					// cutting the motors mid-air.
					send_stabilized_mode_command();
					RCLCPP_WARN(this->get_logger(),
						"[AUTO] AUTO->MANUAL in air: PX4 -> STABILIZED, staying ARMED "
						"(pilot takeover)");
				} else {
					send_arm_command(false);
					send_stabilized_mode_command();
				}
				offboard_state_ = OffboardState::INACTIVE;
			}
			arm_pending_       = false;
			arm_retry_active_  = false;
			arm_failed_        = false;
			landing_z_stopped_ = false;
			auto_mode_active_  = false;
			auto_sub_state_    = AutoSubState::IDLE;
			has_new_mode_cmd_  = false;  // discard any pending A2G re-queue
			post_landing_disarm_guard_ = false;  // manual re-arm is the pilot's call
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
		// Post-landing disarm guard: PX4's RC arm switch is level-triggered — with the
		// pilot's switch still in an ARM position plus residual throttle, PX4 re-arms
		// right after the touchdown DISARM and the vehicle takes off again. Until the
		// A2G transform completes (config leaves UAV), actively disarm any re-arm.
		// Cleared by: A2G completion, an explicit mode_cmd=1, AUTO exit, kill switch.
		if (post_landing_disarm_guard_) {
			if (m4_configuration != M4Configurations::UAV) {
				post_landing_disarm_guard_ = false;   // A2G done — guard no longer needed
			} else if (offboard_state_ == OffboardState::INACTIVE &&
			           pixhawk_state.arming_state ==
			               px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
			           (this->now() - last_guard_disarm_).seconds() > 0.5) {
				last_guard_disarm_ = this->now();
				send_arm_command(false);
				RCLCPP_WARN(this->get_logger(),
					"[AUTO] post-landing guard: PX4 re-armed (RC arm switch/throttle?) "
					"-> DISARM (awaiting A2G)");
			}
		}
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
		return;
	}

	// ────────────────────────────────────────────────────────────────────
	// [④] MANUAL mode
	// ────────────────────────────────────────────────────────────────────
	if (rc_state.arm_switch_state == ArmSwitchState::GROUND_ARM) {
		if (rc_state.uav_switch_state == UAVSwitchState::UAV_DISABLE)
		{// Drive Mode
			// Guard: block A2G transform until PX4 confirms disarmed
			if (m4_configuration == M4Configurations::UAV &&
			    pixhawk_state.arming_state ==
			    px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
			{
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
					"[MANUAL] A2G transform blocked — waiting for PX4 DISARM");
			} else {
				for (const auto& handler : drive_mode_transition_handler)
				{
					if (rc_state.drive_switch_state == handler.state &&
					    m4_configuration != handler.configuration)
					{
						RCLCPP_WARN(this->get_logger(), "%s", handler.message);
						#ifndef DRY_EXECUTION_MODE
							(m4->*handler.action)();
							m4_configuration = m4->get_current_configuration();
						#else
							m4_configuration = handler.configuration;
						#endif
						break;
					}
				}
			}
			#ifndef DRY_EXECUTION_MODE
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
					#ifndef DRY_EXECUTION_MODE
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
		if (m4_configuration != M4Configurations::UAV) {
			RCLCPP_ERROR(this->get_logger(), "Not in UAV CONFIGURATION. Disarming!");
			send_arm_command(false);
			RCLCPP_INFO(this->get_logger(), "Disarm command sent to Pixhawk");
		}
	}

	publish_mode_status();
}

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
