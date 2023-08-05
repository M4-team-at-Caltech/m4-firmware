#include <ros/ros.h>
#include "m4_base.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv,"m4_base_node");
	M4Base m4;
	ros::spin();
	return 0;
}