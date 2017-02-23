// SYSTEM INCLUDES
#include <math.h>

// ROS INCLUDES
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

// C++ PROJECT INCLUDES
#include "lab_3/waypoint_generator.h"


geometry_msgs::PoseStamped generate_waypoint(const double x, const double y, const double phi)
{
	geometry_msgs::PoseStamped pose_stamped;

	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = 0.0;

	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = sin(phi / 2.0);
	pose.orientation.w = cos(phi / 2.0);

	pose_stamped.pose = pose;
	return pose_stamped;
}
