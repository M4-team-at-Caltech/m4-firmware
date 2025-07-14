#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "m4_base.h"

// #define DRY_RUN

struct DriveConfig
{
	int channel_id;
	int stand;
	int crouch1;
	int crouch2;
}drive_config;

struct MIPConfig
{
	int channel_id;
	int drive;
	int mip_transform;
	int mip_ready;
}mip_config;

struct UAVConfig
{
	int channel_id;
	int drive;
	int uav_transform;
	int uav_ready;
}uav_config;

struct XVelConfig
{
	int channel_id;
	int min;
	int zero;
	int max;
}xvel_config;

struct YVelConfig
{
	int channel_id;
	int min;
	int zero;
	int max;
}yvel_config;

struct ArmConfig
{
	int channel_id;
	int disarm;
	int ground_mode_arm;
	int aerial_mode_arm;
}arm_config;

enum DriveMode
{
	MODE_STAND,
	MODE_CROUCH1,
	MODE_CROUCH2
}drive_channel_state;

enum MIPMode
{
	MODE_MIP_DISABLE,
	MODE_MIP_TRANSFORM,
	MODE_MIP_READY
}mip_channel_state;

enum UAVMode
{
	MODE_UAV_DISABLE,
	MODE_UAV_TRANSFORM
}uav_channel_state;

enum ArmMode
{
	MODE_DISARM,
	MODE_GROUND_ARM,
	MODE_AERIAL_ARM
}arm_channel_state;

enum M4Mode
{
	MODE_DRIVE,
	MODE_MIP,
	MODE_UAV
}current_mode;

float xvel_state;
float yvel_state;
float velocity_deadzone;

mavros_msgs::State pixhawk_state;

bool rc_state_available;
bool pixhawk_state_available;
bool drive_mode;
bool teensy_ack;


void rc_subscriber_callback(const mavros_msgs::RCIn &rc_msg)
{
	// Drive Channel
	if (rc_msg.channels[drive_config.channel_id-1] == drive_config.stand) {
		drive_channel_state = DriveMode::MODE_STAND;
	} else if (rc_msg.channels[drive_config.channel_id-1] == drive_config.crouch1) {
		drive_channel_state = DriveMode::MODE_CROUCH1;
	} else if (rc_msg.channels[drive_config.channel_id-1] == drive_config.crouch2) {
		drive_channel_state = DriveMode::MODE_CROUCH2;
	}

	/// MIP CHANNEL NOT SET!
	// MIP Channel
	// if (rc_msg.channels[mip_config.channel_id-1] == mip_config.drive) {
	// 	mip_channel_state = MIPMode::MODE_MIP_DISABLE;
	// } else if (rc_msg.channels[mip_config.channel_id-1] == mip_config.mip_transform) {
	// 	mip_channel_state = MIPMode::MODE_MIP_TRANSFORM;
	// } else if (rc_msg.channels[mip_config.channel_id-1] == mip_config.mip_ready) {
	// 	mip_channel_state = MIPMode::MODE_MIP_READY;
	// }

	// UAV Channel
	if (rc_msg.channels[uav_config.channel_id-1] == uav_config.drive) {
		uav_channel_state = UAVMode::MODE_UAV_DISABLE;
	} else if (rc_msg.channels[uav_config.channel_id-1] == uav_config.uav_transform) {
		uav_channel_state = UAVMode::MODE_UAV_TRANSFORM;
	}

	// ARM Channel
	if (rc_msg.channels[arm_config.channel_id-1] == arm_config.disarm) {
		arm_channel_state = ArmMode::MODE_DISARM;
	} else if (rc_msg.channels[arm_config.channel_id-1] == arm_config.ground_mode_arm) {
		arm_channel_state = ArmMode::MODE_GROUND_ARM;
	} else if (rc_msg.channels[arm_config.channel_id-1] == arm_config.aerial_mode_arm) {
		arm_channel_state = ArmMode::MODE_AERIAL_ARM;
	}

	xvel_state = float((xvel_config.zero - rc_msg.channels[xvel_config.channel_id-1])) / (xvel_config.max - xvel_config.zero);
	yvel_state = float(rc_msg.channels[yvel_config.channel_id-1] - yvel_config.zero) / (yvel_config.max - yvel_config.zero);

	if abs(xvel_state) <= velocity_deadzone
		xvel_state = 0;
	if abs(yvel_state) <= velocity_deadzone
		yvel_state = 0;

	rc_state_available = true;
}

void pixhawk_state_subscriber_callback(mavros_msgs::State state_msg)
{
	pixhawk_state = state_msg;
	pixhawk_state_available = true;
}

void init_configs(std::string config_path)
{
	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + config_path;
	YAML::Node config = YAML::LoadFile(configFilePath);

	drive_config.channel_id = config["drive_mode"]["channel_id"].as<int>();
	drive_config.stand = config["drive_mode"]["stand"].as<int>();
	drive_config.crouch1 = config["drive_mode"]["crouch1"].as<int>();
	drive_config.crouch2 = config["drive_mode"]["crouch2"].as<int>();

	// MIP CHANNEL NOT SET!
	// mip_config.channel_id = config["mip_mode"]["channel_id"].as<int>();
	// mip_config.drive = config["mip_mode"]["drive"].as<int>();
	// mip_config.mip_transform = config["mip_mode"]["mip_transform"].as<int>();
	// mip_config.mip_ready = config["mip_mode"]["mip_ready"].as<int>();

	uav_config.channel_id = config["uav_mode"]["channel_id"].as<int>();
	uav_config.drive = config["uav_mode"]["drive"].as<int>();
	uav_config.uav_transform = config["uav_mode"]["uav_transform"].as<int>();

	xvel_config.channel_id = config["x_velocity"]["channel_id"].as<int>();
	xvel_config.min = config["x_velocity"]["min"].as<int>();
	xvel_config.max = config["x_velocity"]["max"].as<int>();
	xvel_config.zero = config["x_velocity"]["zero"].as<int>();
	
	yvel_config.channel_id = config["y_velocity"]["channel_id"].as<int>();
	yvel_config.min = config["y_velocity"]["min"].as<int>();
	yvel_config.max = config["y_velocity"]["max"].as<int>();
	yvel_config.zero = config["y_velocity"]["zero"].as<int>();

	velocity_deadzone = config["velocity_deadzone"].as<float>();

	arm_config.channel_id = config["arming"]["channel_id"].as<int>();
	arm_config.disarm = config["arming"]["disarm"].as<int>();
	arm_config.ground_mode_arm = config["arming"]["ground_mode_arm"].as<int>();
	arm_config.aerial_mode_arm = config["arming"]["aerial_mode_arm"].as<int>();

	drive_mode = false;
	rc_state_available = false;
	pixhawk_state_available = false;
}

int main(int argc, char **argv)
{
	init_configs("/config/rc_channel_config.yaml");

	ros::init(argc, argv, "m4_controller_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, rc_subscriber_callback);
	ros::Subscriber pixhawk_state_sub = nh.subscribe("/mavros/state", 1, pixhawk_state_subscriber_callback);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::Publisher rc_override_pub = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

	// std::vector<int> active_joint_ids = {1, 2, 3, 4, 5, 6, 7, 8};
	std::vector<int> active_joint_ids = {1,2,3,4,5,6,7,8};
	std::vector<int> active_wheel_ids = {9,10};
	#ifndef DRY_RUN
		M4Base m4(active_joint_ids, active_wheel_ids, false);
	#endif

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	#ifndef DRY_RUN
		m4.sit();
		std::cout<<"Sit complete!\n";
		while(m4.get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));
		std::cout<<"Sit wait complete!\n";
	#endif
	
	#ifndef DRY_RUN
		m4.stand();
		while(m4.get_current_configuration() != M4Configurations::STAND) std::this_thread::sleep_for(std::chrono::milliseconds(100));
	#endif

	ros::Rate loop_rate(200);

	M4Configurations current_configuration=M4Configurations::INIT;
	#ifdef DRY_RUN
		current_configuration = M4Configurations::STAND;
	#endif

	while (ros::ok())
	{

		if(rc_state_available)
		{
			ROS_WARN_ONCE("M4 Controller Active!");
			// printf("DC: %d\tMC: %d\tUC: %d\tAC: %d\tXV: %.2f\tYV: %.2f\r", 
			// drive_channel_state,
			// mip_channel_state,
			// uav_channel_state,
			// arm_channel_state,
			// xvel_state,
			// yvel_state
			// );

			fflush(stdout);
			rc_state_available = false;

			switch(arm_channel_state)
			{
				// If disarmed do nothing
				case ArmMode::MODE_DISARM:
					m4.drive(0,0);
					drive_mode = false;
					break;

				// If armed for ground mode
				case ArmMode::MODE_GROUND_ARM:
					#ifndef DRY_RUN
						current_configuration = m4.get_current_configuration();
					#endif

					// Check for drive mode
					if(mip_channel_state == MIPMode::MODE_MIP_DISABLE && uav_channel_state == UAVMode::MODE_UAV_DISABLE)
					{
						current_mode = M4Mode::MODE_DRIVE;
						
						// Make any required transformations
						switch(drive_channel_state)
						{
							case DriveMode::MODE_STAND:
								// Do nothing if already in stand configuration
								if(current_configuration == M4Configurations::STAND)
								{
									drive_mode = true;
									break;
								}

								else // Move to stand configuration
								{
									ROS_WARN("Move to stand");
									
									#ifndef DRY_RUN
										m4.stand();
									#else
										current_configuration = M4Configurations::STAND;
									#endif

									drive_mode = true;
								}

								break;

							case DriveMode::MODE_CROUCH1:
								// Do nothing if already in crouch1 configuration
								if(current_configuration == M4Configurations::CROUCH1)
								{
									break;
								}

								else // Move to crouch1 configuration
								{
									std::this_thread::sleep_for(std::chrono::milliseconds(500));
									ROS_WARN("Move to crouch1");
										
									#ifndef DRY_RUN
										m4.crouch1();
									#else
										current_configuration = M4Configurations::CROUCH1;
									#endif

									drive_mode = true;
								}				

								break;

							case DriveMode::MODE_CROUCH2:
								// Do nothing if already in crouch 2 configuration
								if(current_configuration == M4Configurations::CROUCH2)
								{
									break;
								}

								else // Move to crouch2 configuration
								{
									ROS_WARN("Move to crouch2");

									#ifndef DRY_RUN
										m4.crouch2();
									#else
										current_configuration = M4Configurations::CROUCH2;
									#endif

									drive_mode = true;
								}

								break;
						}

						if(drive_mode)
						{
							m4.drive(xvel_state,yvel_state);
						}
						else
						{
							m4.drive(0,0);
						}
					}

					// MIP MODE NOT IMPLEMENTED! 
					// Check for MIP mode
					// else if(mip_channel_state != MIPMode::MODE_MIP_DISABLE && (current_mode == M4Mode::MODE_DRIVE || current_mode == M4Mode::MODE_MIP))
					// {
					// 	current_mode = M4Mode::MODE_MIP;
						
					// 	// Make any transformations if needed
					// 	switch(mip_channel_state)
					// 	{
					// 		case MIPMode::MODE_MIP_TRANSFORM:
					// 			// Do nothing if already in mip configuration
					// 			if(current_configuration == M4Configurations::MIP)
					// 			{
					// 				if(!teensy_cmd_flag == TeensyCmd::STOP)
                    //                 {
                    //                     teensy_cmd(teensy_cmd_pub, TeensyCmd::STOP);
					// 					drive_mode = false;
                    //                 }
					// 				break;
					// 			}

					// 			// else // Move to mip configuration
					// 			// {
					// 			// 	teensy_cmd(teensy_cmd_pub, TeensyCmd::STOP);
					// 			// 	std::this_thread::sleep_for(std::chrono::milliseconds(500));
									
					// 			// 	ROS_WARN("Move to mip");
									
					// 			// 	if(current_configuration == M4Configurations::STAND || current_configuration == M4Configurations::CROUCH1 || current_configuration == M4Configurations::CROUCH2 )
					// 			// 		teensy_cmd(teensy_cmd_pub, TeensyCmd::WHEEL_OUT);


					// 			// 	m4.mip();

					// 			// 	teensy_cmd(teensy_cmd_pub, TeensyCmd::STOP);
					// 			// 	std::this_thread::sleep_for(std::chrono::milliseconds(500));
					// 			// 	teensy_cmd(teensy_cmd_pub, TeensyCmd::TRANSFORM_MIP);
					// 			// }

					// 			break;

					// 		case MIPMode::MODE_MIP_READY:
					// 			// Move to MIP configuration if not already in it
					// 			if(current_configuration != M4Configurations::MIP)
					// 			{
					// 				teensy_cmd(teensy_cmd_pub, TeensyCmd::STOP);
					// 				std::this_thread::sleep_for(std::chrono::milliseconds(500));
									
					// 				ROS_WARN("Move to mip");
									
					// 				if(current_configuration == M4Configurations::STAND || current_configuration == M4Configurations::CROUCH1 || current_configuration == M4Configurations::CROUCH2 )
					// 					teensy_cmd(teensy_cmd_pub, TeensyCmd::WHEEL_OUT);

					// 				m4.mip();

					// 				teensy_cmd(teensy_cmd_pub, TeensyCmd::STOP);
					// 				std::this_thread::sleep_for(std::chrono::milliseconds(500));
					// 				teensy_cmd(teensy_cmd_pub, TeensyCmd::TRANSFORM_MIP);
					// 				std::this_thread::sleep_for(std::chrono::milliseconds(500));
					// 				teensy_cmd(teensy_cmd_pub, TeensyCmd::MIP_BALANCE);
					// 			}

					// 			if(!teensy_cmd_flag == TeensyCmd::MIP_BALANCE)
					// 			{
					// 				teensy_cmd(teensy_cmd_pub, TeensyCmd::MIP_BALANCE);
					// 			}

					// 			break;		
					// 	}
					// }
			
					// Check for UAV mode
					else if(uav_channel_state != UAVMode::MODE_UAV_DISABLE && (current_mode == M4Mode::MODE_DRIVE || current_mode == M4Mode::MODE_UAV))
					{
						current_mode = M4Mode::MODE_UAV;

						// Make any transformations if needed
						switch(uav_channel_state)
						{
							case UAVMode::MODE_UAV_TRANSFORM:
								// Do nothing if already in uav configuration
								if(current_configuration == M4Configurations::UAV)
								{
									break;
								}

								else // Move to uav configuration
								{
									m4.drive(0.0,0.0);
									drive_mode = false;
									
									ROS_WARN("Move to uav\n");
									#ifndef DRY_RUN
										m4.uav();
									#else
										current_configuration = M4Configurations::UAV;
									#endif

								}

								break;
						}
					}
			
					break;

				// If armed for aerial mode
				case ArmMode::MODE_AERIAL_ARM:
					#ifndef DRY_RUN
						current_configuration = m4.get_current_configuration();
					#endif

					// Do nothing if armed
					if(pixhawk_state.armed)
					{
						ROS_DEBUG("Armed. Do nothing.");
						break;
					}

					// Override arm command if not in UAV mode and ready (teensy handed control back to pixhawk)
					if((current_configuration != M4Configurations::UAV))
					{
						ROS_ERROR("Not in UAV CONFIGURATION. Disarming!");
						mavros_msgs::CommandBool arm_cmd;
						arm_cmd.request.value = false;
						if(arming_client.call(arm_cmd) && arm_cmd.response.success)
						{
							ROS_INFO("Vehicle disarmed\n");
						}
					}

					break;
			}
			
		}	

		loop_rate.sleep();
		ros::spinOnce();
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	#ifndef DRY_RUN
		m4.sit();
		while(m4.get_current_configuration() != M4Configurations::SIT) std::this_thread::sleep_for(std::chrono::milliseconds(100));
	#endif
}

