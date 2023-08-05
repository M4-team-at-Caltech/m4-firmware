#include <vector>
#include <string>
#include <ros/ros.h>
#include "DynamixelInterface.h"
#include "m4_msgs/M4Cmd.h"

enum M4Commands
{
	STAND,
	CROUCH,
	SIT,
	UAV,
	MIP,
	CUSTOM
};

class M4Base
{
	public:
		M4Base();
		~M4Base();

	private:
		DynamixelPositionInterface dxl;
		ros::NodeHandle nh;
		ros::Publisher joint_state_publisher;
		ros::Subscriber M4Commands_subscriber;

		void status_read_loop();
		void command_callback(m4_msgs::M4Cmd cmd);
		// Transformations
		void stand();
		void crouch();
		void sit();
		void uav();
		void mip();
		void custom_joint_pos(const std::vector<int> &ids, const std::vector<float> &positions);
};