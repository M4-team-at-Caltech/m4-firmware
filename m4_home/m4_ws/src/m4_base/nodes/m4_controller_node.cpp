#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include "m4_base.h"

struct DriveConfig
{
	int channel_id;
	int stand;
	int crouch;
	int sit;
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
	MODE_CROUCH,
	MODE_SIT
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
	MODE_UAV_TRANSFORM,
	MODE_UAV_READY
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

mavros_msgs::State pixhawk_state;

bool rc_state_available;
bool pixhawk_state_available;
bool teensy_running;
bool teensy_control;


void rc_subscriber_callback(const mavros_msgs::RCIn &rc_msg)
{
	// Drive Channel
	if (rc_msg.channels[drive_config.channel_id-1] == drive_config.stand) {
		drive_channel_state = DriveMode::MODE_STAND;
	} else if (rc_msg.channels[drive_config.channel_id-1] == drive_config.crouch) {
		drive_channel_state = DriveMode::MODE_CROUCH;
	} else if (rc_msg.channels[drive_config.channel_id-1] == drive_config.sit) {
		drive_channel_state = DriveMode::MODE_SIT;
	}

	// MIP Channel
	if (rc_msg.channels[mip_config.channel_id-1] == mip_config.drive) {
		mip_channel_state = MIPMode::MODE_MIP_DISABLE;
	} else if (rc_msg.channels[mip_config.channel_id-1] == mip_config.mip_transform) {
		mip_channel_state = MIPMode::MODE_MIP_TRANSFORM;
	} else if (rc_msg.channels[mip_config.channel_id-1] == mip_config.mip_ready) {
		mip_channel_state = MIPMode::MODE_MIP_READY;
	}

	// UAV Channel
	if (rc_msg.channels[uav_config.channel_id-1] == uav_config.drive) {
		uav_channel_state = UAVMode::MODE_UAV_DISABLE;
	} else if (rc_msg.channels[uav_config.channel_id-1] == uav_config.uav_transform) {
		uav_channel_state = UAVMode::MODE_UAV_TRANSFORM;
	} else if (rc_msg.channels[uav_config.channel_id-1] == uav_config.uav_ready) {
		uav_channel_state = UAVMode::MODE_UAV_READY;
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
	drive_config.crouch = config["drive_mode"]["crouch"].as<int>();
	drive_config.sit = config["drive_mode"]["sit"].as<int>();

	mip_config.channel_id = config["mip_mode"]["channel_id"].as<int>();
	mip_config.drive = config["mip_mode"]["drive"].as<int>();
	mip_config.mip_transform = config["mip_mode"]["mip_transform"].as<int>();
	mip_config.mip_ready = config["mip_mode"]["mip_ready"].as<int>();

	uav_config.channel_id = config["uav_mode"]["channel_id"].as<int>();
	uav_config.drive = config["uav_mode"]["drive"].as<int>();
	uav_config.uav_transform = config["uav_mode"]["uav_transform"].as<int>();
	uav_config.uav_ready = config["uav_mode"]["uav_ready"].as<int>();

	xvel_config.channel_id = config["x_velocity"]["channel_id"].as<int>();
	xvel_config.min = config["x_velocity"]["min"].as<int>();
	xvel_config.max = config["x_velocity"]["max"].as<int>();
	xvel_config.zero = config["x_velocity"]["zero"].as<int>();
	
	yvel_config.channel_id = config["y_velocity"]["channel_id"].as<int>();
	yvel_config.min = config["y_velocity"]["min"].as<int>();
	yvel_config.max = config["y_velocity"]["max"].as<int>();
	yvel_config.zero = config["y_velocity"]["zero"].as<int>();

	arm_config.channel_id = config["arming"]["channel_id"].as<int>();
	arm_config.disarm = config["arming"]["disarm"].as<int>();
	arm_config.ground_mode_arm = config["arming"]["ground_mode_arm"].as<int>();
	arm_config.aerial_mode_arm = config["arming"]["aerial_mode_arm"].as<int>();

	teensy_running = false;
	teensy_control = true;
}

void send_teensy_stop()
{
	if(teensy_running)
	{
		ROS_WARN("Sending teensy stop");
		// TODO: Implement stop command to teensy, wait for ack
		teensy_running = false;
	}
}

void send_teensy_vel()
{
	ROS_INFO("Sending teensy velocities");
	// TODO: Implement velocity sending to teensy
}

int main(int argc, char **argv)
{
	init_configs("/config/rc_channel_config.yaml");

	ros::init(argc, argv, "m4_controller_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/mavros/rc/in", 1, rc_subscriber_callback);

	std::vector<int> active_joint_ids = {1, 2, 3, 4};
	// M4Base m4(active_joint_ids, true);

	ros::Rate loop_rate(200);

	M4Configurations current_configuration=M4Configurations::INIT;

	while (ros::ok())
	{
		if(rc_state_available)
		{
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
					send_teensy_stop();
					break;

				// If armed for ground mode
				case ArmMode::MODE_GROUND_ARM:
					// M4Configurations current_configuration = m4.get_current_configuration();

					// Check for drive mode
					if(mip_channel_state == MIPMode::MODE_MIP_DISABLE && uav_channel_state == UAVMode::MODE_UAV_DISABLE)
					{
						current_mode = M4Mode::MODE_DRIVE;
						if(!teensy_control)
						{
							// TODO: Send message to teensy to take back control
							ROS_WARN("Taking back teensy control\n");
							teensy_control = true;
						}
						
						// Make any required transformations
						switch(drive_channel_state)
						{
							case DriveMode::MODE_STAND:
								// Do nothing if already in stand configuration
								if(current_configuration == M4Configurations::STAND)
								{
									teensy_running = true;
									break;
								}

								else // Move to stand configuration
								{
									send_teensy_stop();

									ROS_WARN("Move to stand\n");
									current_configuration = M4Configurations::STAND;
									// m4.stand();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									teensy_running = true;
								}

								break;

							case DriveMode::MODE_CROUCH:
								// Do nothing if already in crouch configuration
								if(current_configuration == M4Configurations::CROUCH)
								{
									teensy_running = true;
									break;
								}

								// If in UAV or MIP configurations move to stand first
								else if(current_configuration == M4Configurations::MIP || current_configuration == M4Configurations::UAV)
								{

									if(teensy_running)
									{
										// TODO: Implement code to stop teensy, wait for ack, set teensy running to false.
										teensy_running = false;
									}

									ROS_WARN("Move to stand\n");
									current_configuration = M4Configurations::STAND;
									// m4.stand();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									teensy_running = true;
								}

								else // Move to crouch configuration
								{
									send_teensy_stop();

									ROS_WARN("Move to crouch\n");
									current_configuration = M4Configurations::CROUCH;
									// m4.crouch();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::CROUCH){loop_rate.sleep(); ros::spinOnce();}
									teensy_running = true;
								}				

								break;

							case DriveMode::MODE_SIT:
								// Do nothing if already in sit configuration
								if(current_configuration == M4Configurations::SIT)
								{
									teensy_running = true;
									break;
								}

								// If in UAV or MIP configurations move to stand first
								else if(current_configuration == M4Configurations::MIP || current_configuration == M4Configurations::UAV)
								{
									send_teensy_stop();

									ROS_WARN("Move to stand\n");
									current_configuration = M4Configurations::STAND;
									// m4.stand();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									teensy_running = true;
								}

								else // Move to sit configuration
								{
									send_teensy_stop();

									ROS_WARN("Move to sit\n");
									current_configuration = M4Configurations::SIT;
									// m4.sit();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::SIT){loop_rate.sleep(); ros::spinOnce();}
									teensy_running = true;
								}

								break;
						}

						if(teensy_running)
						{
							send_teensy_vel();
						}
					}

					// Check for MIP mode
					else if(mip_channel_state != MIPMode::MODE_MIP_DISABLE && (current_mode == M4Mode::MODE_DRIVE || current_mode == M4Mode::MODE_MIP))
					{
						current_mode = M4Mode::MODE_MIP;
						if(!teensy_control)
						{
							// TODO: Send message to teensy to take back control
							ROS_WARN("Taking back teensy control\n");
							teensy_control = true;
						}
						
						// Make any transformations if needed
						switch(mip_channel_state)
						{
							case MIPMode::MODE_MIP_TRANSFORM:
								// Do nothing if already in mip configuration
								if(current_configuration == M4Configurations::MIP)
								{
									send_teensy_stop();
									break;
								}

								else // Move to mip configuration
								{
									send_teensy_stop();

									if(current_configuration != M4Configurations::STAND)
									{
										ROS_WARN("Move to stand\n");
										current_configuration = M4Configurations::STAND;
										// m4.stand();
										// // Wait until it completes
										// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									}

									ROS_WARN("Move to mip\n");
									current_configuration = M4Configurations::MIP;
									// m4.mip();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::MIP){loop_rate.sleep(); ros::spinOnce();}
								}

								break;

							case MIPMode::MODE_MIP_READY:
								// Move to MIP configuration if not already in it
								if(current_configuration != M4Configurations::MIP)
								{
									send_teensy_stop();

									if(current_configuration != M4Configurations::STAND)
									{
										ROS_WARN("Move to stand\n");
										current_configuration = M4Configurations::STAND;
										// m4.stand();
										// // Wait until it completes
										// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									}

									ROS_WARN("Move to mip\n");
									current_configuration = M4Configurations::MIP;
									// m4.mip();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::MIP){loop_rate.sleep(); ros::spinOnce();}	
								}
								
								if(!teensy_running)
								{
									// TODO: Send command to initializa MIP mode, wait for ack
									teensy_running = true;
								}

								break;		
						}

						if(teensy_running)
						{
							send_teensy_vel();
						}
					}
			
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
									send_teensy_stop();
									if(!teensy_control)
									{
										// TODO: Send message to teensy to take back control
										ROS_WARN("Taking back teensy control\n");
										teensy_control = true;
									}

									break;
								}

								else // Move to uav configuration
								{
									send_teensy_stop();
									if(!teensy_control)
									{
										// TODO: Send message to teensy to take back control
										ROS_WARN("Taking back teensy control\n");
										teensy_control = true;
									}

									if(current_configuration != M4Configurations::STAND)
									{
										ROS_WARN("Move to stand\n");
										current_configuration = M4Configurations::STAND;
										// m4.stand();
										// // Wait until it completes
										// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									}

									ROS_WARN("Move to uav\n");
									current_configuration = M4Configurations::UAV;
									// m4.uav();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::UAV){loop_rate.sleep(); ros::spinOnce();}
								}

								break;

							case UAVMode::MODE_UAV_READY:
								// Move to MIP configuration if not already in it
								if(current_configuration != M4Configurations::UAV)
								{
									send_teensy_stop();

									if(current_configuration != M4Configurations::STAND)
									{
										ROS_WARN("Move to stand\n");
										current_configuration = M4Configurations::STAND;
										// m4.stand();
										// // Wait until it completes
										// while(!m4.get_current_configuration() == M4Configurations::STAND){loop_rate.sleep(); ros::spinOnce();}
									}

									ROS_WARN("Move to uav\n");
									current_configuration = M4Configurations::UAV;
									// m4.uav();
									// // Wait until it completes
									// while(!m4.get_current_configuration() == M4Configurations::UAV){loop_rate.sleep(); ros::spinOnce();}	
								}
								
								if(teensy_control)
								{
									// TODO: Tell teensy to switch control to pixhawk, wait for ack
									ROS_WARN("Switching control to pixhawk\n");
									teensy_control = false;
								}

								break;		
						}

						if(!teensy_control)
							ROS_INFO("Pixhawk in control!");
					}
			}
			
		}	

		loop_rate.sleep();
		ros::spinOnce();
	}
	
}