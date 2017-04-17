// SYSTEM INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


// C++ PROJECT INCLUDES
#include "eecs376_ps1/stdr_commands.h"


#define PI 3.141592653589793238462643383279502884

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stdr_commander");
	ros::NodeHandle n;
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);

	double forward_speed = 1.0;
	double rotate_time = 3.0;
	double yaw_rate = PI / (2 * rotate_time);
	double publish_rate_in_sec = 0.01;
	ros::Rate timer(1.0 / publish_rate_in_sec);

	geometry_msgs::Twist cmd;

	// allow ros connections to initalize by publishing command with no movement.
	move(publish_rate_in_sec, 0.1, timer, twist_pub, zero_twist_command(cmd));


	// navigate the maze to the top left corner
	move_forward_x(forward_speed, publish_rate_in_sec, 4.0, timer, twist_pub, cmd);

	rotate_about_z(yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 2.5, timer, twist_pub, cmd);

	rotate_about_z(-yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 4.0, timer, twist_pub, cmd);

	rotate_about_z(yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 2.7, timer, twist_pub, cmd);

	rotate_about_z(yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 6.5, timer, twist_pub, cmd);

	rotate_about_z(-yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 1.0, timer, twist_pub, cmd);

	rotate_about_z(yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 1.5, timer, twist_pub, cmd);

	rotate_about_z(-yaw_rate, publish_rate_in_sec, rotate_time, timer, twist_pub, cmd);

	move_forward_x(forward_speed, publish_rate_in_sec, 6.0, timer, twist_pub, cmd);

	// tell the robot to stop moving.
	move(publish_rate_in_sec, 0.1, timer, twist_pub, zero_twist_command(cmd));
	return 0;
}
