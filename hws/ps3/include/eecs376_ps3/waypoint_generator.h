#pragma once
#ifndef EECS376_PS3_WAYPOINT_GENERATOR_H
#define EECS376_PS3_WAYPOINT_GENERATOR_H

// SYSTEM INCLUDES


// ROS INCLUDES
#include <geometry_msgs/PoseStamped.h>


// C++ PROJECT INCLUDES

/**
 * This function takes a pair of x, y coordinates and an angle phi about the z axis, and comverts
 * them into a valid PoseStamped message. This message is intended for use in a list of waypoints
 * a "nav_msgs/Path" message, but theoretically should be valid for any 2D geometry.
 */
geometry_msgs::PoseStamped generate_waypoint(const double x, const double y, const double phi);

#endif // end of EECS376_PS3_WAYPOINT_GENERATOR_H
