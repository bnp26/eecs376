#pragma once
#ifndef EECS376_PS2_MOVEMENT_COMMANDS_H
#define EECS376_PS2_MOVEMENT_COMMANDS_H


// SYSTEM INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// C++ PROJECT INCLUDES


/**
 * A function that sets all members of a geometry::Twist object to 0.0.
 */
geometry_msgs::Twist& zero_twist_command(geometry_msgs::Twist& cmd);

/**
 * A function that will publish a geometry::Twist object.
 */
void move(ros::Publisher& publisher, geometry_msgs::Twist& cmd);

/**
 * A function that will zero out a geometry::Twist object, set the Twist object
 * to represent turning about the z axis at rate of speed and publish the message.
 */
void rotate_about_z(double speed, ros::Publisher& publisher, geometry_msgs::Twist& cmd);

/**
 * A function that will zero out a geometry::Twist object, set the Twist object
 * to represent moving forward in the x direction at rate of speed, and publish the message.
 */
void move_forward_x(double speed, ros::Publisher& publisher, geometry_msgs::Twist& cmd);

#endif // end of EECS376_MOVEMENT_COMMANDS_H
