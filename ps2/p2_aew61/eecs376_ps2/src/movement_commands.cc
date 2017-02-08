// SYSTEM INCLUDES


// C++ PROJECT INCLUDES
#include "eecs376_ps2/movement_commands.h"


geometry_msgs::Twist& zero_twist_command(geometry_msgs::Twist& cmd)
{
	// set linear components to 0
	cmd.linear.x = 0.0;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;

	// set angular components to 0
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = 0.0;

	// safe to return reference
	return cmd;
}

void move(ros::Publisher& publisher, geometry_msgs::Twist& cmd)
{
	publisher.publish(cmd);
}

void rotate_about_z(double speed, ros::Publisher& publisher, geometry_msgs::Twist& cmd)
{
	zero_twist_command(cmd);
	cmd.angular.z = speed;

	move(publisher, cmd);
}

void move_forward_x(double speed, ros::Publisher& publisher, geometry_msgs::Twist& cmd)
{
	zero_twist_command(cmd);
	cmd.linear.x = speed;

	move(publisher, cmd);
}
