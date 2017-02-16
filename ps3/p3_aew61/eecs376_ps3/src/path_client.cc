// SYSTEM INCLUDES
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>

// C++ PROJECT INCLUDES
#include "eecs376_ps3/PathSrv.h"
#include "eecs376_ps3/waypoint_generator.h"

#define PI 3.141592653589793238462643383279502884

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hw3_path_client");
	ros::NodeHandle handle;
	ros::ServiceClient client = handle.serviceClient<eecs376_ps3::PathSrv>("path_service");

	while(!client.exists() && ros::ok())
	{
		ROS_INFO("waiting for service...");
		ros::Duration(1.0).sleep();
	}
	if(client.exists())
	{
		ROS_INFO("connected to client!");

		eecs376_ps3::PathSrv path_srv;

		// populate the path of waypoints. Note that there is partial feedback, but only for spinning
		path_srv.request.nav_path.poses.push_back(generate_waypoint(3.5, 0.0, PI / 2));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(3.5, 3.2, 0.0));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(7.0, 3.5, PI / 2));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(7.0, 5.25, PI));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(1.5, 5.2, PI / 2));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(1.5, 6.1, PI));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(0.1, 6.1, PI / 2));
		path_srv.request.nav_path.poses.push_back(generate_waypoint(0.1, 15.0, 3 * PI / 2));

		client.call(path_srv);
	}

	return 0;
}
