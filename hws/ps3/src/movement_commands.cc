// SYSTEM INCLUDES


// C++ PROJECT INCLUDES
#include "eecs376_ps3/movement_commands.h"


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

double move(double dt, double total_time, ros::Rate& timer,
			ros::Publisher& publisher, geometry_msgs::Twist& cmd)
{
	double elapsed_time = 0.0;

	while(elapsed_time < total_time)
	{
		publisher.publish(cmd);
		elapsed_time += dt;
		timer.sleep();
	}

	return elapsed_time;
}

double rotate_about_z(double speed, double dt, double total_time, ros::Rate& timer,
					  ros::Publisher& publisher, geometry_msgs::Twist& cmd)
{
	zero_twist_command(cmd);
	cmd.angular.z = speed;

	return move(dt, total_time, timer, publisher, cmd);
}

double move_forward_x(double speed, double dt, double total_time, ros::Rate& timer,
					  ros::Publisher& publisher, geometry_msgs::Twist& cmd)
{
	zero_twist_command(cmd);
	cmd.linear.x = speed;

	return move(dt, total_time, timer, publisher, cmd);
}
