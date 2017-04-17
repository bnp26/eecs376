#pragma once
#ifndef EECS376_PS1_STDR_COMMANDS_H
#define EECS376_PS1_STDR_COMMANDS_H


// SYSTEM INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// C++ PROJECT INCLUDES


/**
 * A function that sets all members of a geometry::Twist object to 0.0.
 */
geometry_msgs::Twist& zero_twist_command(geometry_msgs::Twist& cmd);

/**
 * A function that will publish a geometry::Twist object for a total amount
 * of time in increments of dt. It is crucial that the rate of the ros::Rate
 * object is 1.0 / dt.
 */
double move(double dt, double total_time, ros::Rate& timer,
			ros::Publisher& publisher, geometry_msgs::Twist& cmd);

/**
 * A function that will zero out a geometry::Twist object, set the Twist object
 * to represent turning about the z axis at rate of speed, for a certain amount
 * of time in increments of dt. It is crucial that the rate of the ros::Rate
 * object is 1.0 / dt.
 */
double rotate_about_z(double speed, double dt, double total_time, ros::Rate& timer, 
					  ros::Publisher& publisher, geometry_msgs::Twist& cmd);

/**
 * A function that will zero out a geometry::Twist object, set the Twist object
 * to represent moving forward in the x direction at rate of speed, for a certain
 * amount of time in increments of dt. It is crucial that the rate of the
 * ros::Rate object is 1.0 / dt.
 */
double move_forward_x(double speed, double dt, double total_time, ros::Rate& timer,
					  ros::Publisher& publisher, geometry_msgs::Twist& cmd);

#endif // end of EECS376_PS1_COMMANDS_H
