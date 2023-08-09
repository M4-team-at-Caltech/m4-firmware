#include <string>
#include <vector>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
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

enum DriveMode
{
	MODE_STAND,
	MODE_CROUCH,
	MODE_SIT
}drive_channel_state;

enum MIPMode
{
	MODE_MIP_DEFAULT,
	MODE_MIP_TRANSFORM,
	MODE_MIP_READY
}mip_channel_state;

enum UAVMode
{
	MODE_UAV_DEFAULT,
	MODE_UAV_TRANSFORM,
	MODE_UAV_READY
}uav_channel_state;

float xvel_state;
float yvel_state;

bool state_available;

void rc_subscriber_callback(const mavros_msgs::RCIn &rc_msg)
{
	if (rc_msg.channels[drive_config.channel_id-1] == drive_config.stand) {
		drive_channel_state = DriveMode::MODE_STAND;
	} else if (rc_msg.channels[drive_config.channel_id-1] == drive_config.crouch) {
		drive_channel_state = DriveMode::MODE_CROUCH;
	} else if (rc_msg.channels[drive_config.channel_id-1] == drive_config.sit) {
		drive_channel_state = DriveMode::MODE_SIT;
	} else {
		ROS_WARN("\nChannel %d [DRIVE] Unknown state!\n", drive_config.channel_id);
	}

	if (rc_msg.channels[mip_config.channel_id-1] == mip_config.drive) {
		mip_channel_state = MIPMode::MODE_MIP_DEFAULT;
	} else if (rc_msg.channels[mip_config.channel_id-1] == mip_config.mip_transform) {
		mip_channel_state = MIPMode::MODE_MIP_TRANSFORM;
	} else if (rc_msg.channels[mip_config.channel_id-1] == mip_config.mip_ready) {
		mip_channel_state = MIPMode::MODE_MIP_READY;
	} else {
		ROS_WARN("\nChannel %d [MIP] Unknown state!\n", mip_config.channel_id);
	}

	if (rc_msg.channels[uav_config.channel_id-1] == uav_config.drive) {
		uav_channel_state = UAVMode::MODE_UAV_DEFAULT;
	} else if (rc_msg.channels[uav_config.channel_id-1] == uav_config.uav_transform) {
		uav_channel_state = UAVMode::MODE_UAV_TRANSFORM;
	} else if (rc_msg.channels[uav_config.channel_id-1] == uav_config.uav_ready) {
		uav_channel_state = UAVMode::MODE_UAV_READY;
	} else {
		ROS_WARN("\nChannel %d [UAV] Unknown state!\n", uav_config.channel_id);
	}

	xvel_state = float((xvel_config.zero - rc_msg.channels[xvel_config.channel_id-1])) / (xvel_config.max - xvel_config.zero);
	yvel_state = float(rc_msg.channels[yvel_config.channel_id-1] - yvel_config.zero) / (yvel_config.max - yvel_config.zero);

	state_available = true;
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
}

int main(int argc, char **argv)
{
	init_configs("/config/rc_channel_config.yaml");

	ros::init(argc, argv, "m4_controller_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("/mavros/rc/in", 1, rc_subscriber_callback);

	std::vector<int> active_joint_ids = {1, 2, 3, 4};
	//M4Base m4(active_joint_ids, true);

	ros::Rate loop_rate(200);

	while (ros::ok())
	{
		if(state_available)
		{
			printf("DC: %d\tMC: %d\tUC: %d\tXV: %.2f\tYV: %.2f\r", 
			drive_channel_state,
			mip_channel_state,
			uav_channel_state,
			xvel_state,
			yvel_state
			);

			fflush(stdout);
		}	

		state_available = false;
		loop_rate.sleep();
		ros::spinOnce();
	}
	
}