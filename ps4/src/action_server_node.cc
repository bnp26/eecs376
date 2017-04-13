// SYSTEM INCLDUES
#include <string>


// ROS INCLUDES
#include <ros/ros.h>


// C++ PROJECT INCLUDES
#include "eecs376_ps4/ActionServer.h"


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_action_server");
	const std::string action_topic = "path_action_server";
	const double linear_vel = 0.5;
	const double yaw_rate = 0.4;
	const double sample_dt = 0.01;
	const std::string publish_topic = "/robot0/cmd_vel";


	ActionServer server(action_topic, linear_vel, yaw_rate,
						sample_dt, publish_topic);

	ros::spin();

	return 0;
}
