#pragma once
#ifndef EECS376_PS3_PATHEXECUTOR_H
#define EECS376_PS3_PATHEXECUTOR_H


// SYSTEM INCLUDES
#include <utility>
#include <vector>


// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


// C++ PROJECT INCLUDES


/**
 * A class that is responsible for consuming a list of waypoints and producing a list of <theta, x> pairs that correspond to the semi-feedback
 * control loop this code implements.
 */
class PathExecutor
{
public:

	// Constructor
	PathExecutor();

	// Destructor. Virtual for inheritance just for the sake of argument.
	virtual ~PathExecutor();

	const static double PI = 3.141592653589793238462643383279502884;

	/**
	 * This member function is responsible for converting a list of waypoints into a list of commands that must be parsed
	 * into robot messages.
	 */
	std::vector<std::pair<double, double> > generate_commands(const std::vector<geometry_msgs::PoseStamped>& waypoints);

protected:
private:

	/**
	 * A member function that returns a quaternion in the form of (x, y, z, w) = (0, 0, 0, 1)
	 */
	geometry_msgs::Quaternion zero_quaternion();

	/**
	 * A member function that returns a pose of (position = (0, 0, 0), orientation = zero_quaternion())
	 */
	geometry_msgs::Pose zero_pose();

	/**
	 * A member function that converts a quaternion to a planar angle about the z axis
	 */
	const double convert_quaternion_to_angle(const geometry_msgs::Quaternion& q);

	/**
	 * A member function to convert a planar angle about the z axis into a quaternion
	 */
	const geometry_msgs::Quaternion convert_angle_to_quaternion(const double phi);

	/**
	 * A member function that computes the shortest angle (why turn 3 * PI / 2 raidans when you can turn -PI / 2 radians?)
	 */
	double min_spin(const double angle);

	/**
	 * A member function that computes the angle difference (about the z axis) between two quaternions
	 */
	double compute_yaw(const geometry_msgs::Pose& current_pose, const geometry_msgs::Quaternion& next_orientation);

	/**
	 * A member function that computes the angle difference (about the z axis) between a given pose and destination coordinates
	 */
	double compute_yaw(const geometry_msgs::Pose& current_pose, const geometry_msgs::Point& to_point_at);

	/**
	 * A member function that computes the distance between two points. This is always positive because
	 * the robot is assumed to only drive forwards.
	 */
	double compute_distance(const geometry_msgs::Point& start, const geometry_msgs::Point& stop);

};


#endif // end of EECS376_PS3_PATHEXECUTOR_H
