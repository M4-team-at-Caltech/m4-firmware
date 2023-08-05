#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "yaml-cpp/yaml.h"
#include "m4_base.h"
#include "m4_msgs/M4Cmd.h"

#define deg2rad(x) ((x)*M_PI/180.0)
#define rad2deg(x) ((x)*180.0/M_PI)

M4Base::M4Base()
{
	std::string configFilePath = std::string(PROJECT_ROOT_DIR) + "/config/robot_config.yaml";
	YAML::Node config = YAML::LoadFile(configFilePath);

	joint_state_publisher = nh.advertise<sensor_msgs::JointState>("/m4/joint_states", 10);
	M4Commands_subscriber = nh.subscribe("/m4/cmd", 10, &M4Base::command_callback, this);

	dxl.init(config["interface_port"].as<std::string>().c_str(), config["baud_rate"].as<int>(), config["active_motor_ids"].as<std::vector<int>>());
}

M4Base::~M4Base()
{
	dxl.stop();
}

void M4Base::status_read_loop()
{
	ros::Rate loop_rate(200);
    while(ros::ok())
    {
		DynamixelStatus joint_states = dxl.read_status();
		sensor_msgs::JointState joint_state_msg;
		joint_state_msg.header.stamp = ros::Time::now();
		joint_state_msg.name = joint_states.joint_names;
		
		std::vector<double> joint_state_msg_position;
		std::vector<double> joint_state_msg_velocity;
		std::vector<double> joint_state_msg_effort;

		for(int i = 0; i < joint_states.joint_names.size(); i++)
		{
			// Convert from deg, deg/s to rad, rad/s
			joint_state_msg_position.push_back(deg2rad(joint_states.position[i]));
            joint_state_msg_velocity.push_back(deg2rad(joint_states.velocity[i]));
            joint_state_msg_effort.push_back(joint_states.current[i]);
		}

		joint_state_msg.position = joint_state_msg_position;
		joint_state_msg.velocity = joint_state_msg_velocity;
		joint_state_msg.effort = joint_state_msg_effort;

        joint_state_publisher.publish(joint_state_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void M4Base::command_callback(m4_msgs::M4Cmd cmd)
{
	switch (cmd.commandID)
	{
		case M4Commands::STAND:
			stand();
			break;
		case M4Commands::CROUCH:
			crouch();
            break;
		case M4Commands::SIT:
			sit();
			break;
		case M4Commands::UAV:
			uav();
			break;
		case M4Commands::MIP:
			mip();
            break;
		case M4Commands::CUSTOM:
			custom_joint_pos(cmd.ids, std::vector<float>(cmd.joint_positions.begin(), cmd.joint_positions.end()));
			break;		
		default:
			ROS_WARN("Unknown command ID: %d", cmd.commandID);
			break;
	}
}

void M4Base::custom_joint_pos(const std::vector<int> &ids, const std::vector<float> &positions)
{
	dxl.write_position(ids, positions);
}

void M4Base::stand()
{

}

void M4Base::crouch()
{

}

void M4Base::sit()
{

}

void M4Base::uav()
{

}

void M4Base::mip()
{

}

