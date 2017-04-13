#pragma once
#ifndef EECS376_PS4_PATHEXECUTOR_H
#define EECS376_PS4_PATHEXECUTOR_H


// SYSTEM INCLUDES
#include <string>
#include <utility>
#include <vector>


// ROS INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Bool.h>


// C++ PROJECT INCLUDES
#include <eecs376_ps4/PathAction.h>

/**
 * A class that is responsible for consuming a list of waypoints and producing a list of <theta, x> pairs that correspond to the semi-feedback
 * control loop this code implements.
 */
class PathExecutor
{
public:

	// Constructor
	PathExecutor(const double linear_vel, const double _yaw_rate,
				 const double sample_dt, std::string pub_topic,
				 ros::NodeHandle* p_handle,
				 actionlib::SimpleActionServer<eecs376_ps4::PathAction>* p_action_server);

	// Destructor. Virtual for inheritance just for the sake of argument.
	virtual ~PathExecutor();

	const static double PI = 3.141592653589793238462643383279502884;

	/**
	 * This member function is responsible for converting a list of waypoints into a list of commands that must be parsed
	 * into robot messages.
	 */
	std::vector<std::pair<double, double> > generate_commands(const std::vector<geometry_msgs::Pose>& waypoints);

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


	void Execute_Command(const std::pair<double, double>& cmd);


	void Send_Zero_Command();


	void alarm_callback(const std_msgs::Bool& alarm_msg);


protected:
private:

	/**
 	* A function that sets all members of a geometry::Twist object to 0.0.
 	*/
	geometry_msgs::Twist& zero_twist_command(geometry_msgs::Twist& cmd);

	/**
 	* A function that will publish a geometry::Twist object for a total amount
 	* of time in increments of dt. It is crucial that the rate of the ros::Rate
 	* object is 1.0 / dt.
 	*/
	double move(double dt, double total_time, geometry_msgs::Twist& cmd, bool override);

	/**
	 * A function that will zero out a geometry::Twist object, set the Twist object
	 * to represent turning about the z axis at rate of speed, for a certain amount
	 * of time in increments of dt. It is crucial that the rate of the ros::Rate
	 * object is 1.0 / dt.
	 */
	double rotate_about_z(double speed, double dt, double total_time, geometry_msgs::Twist& cmd);

	/**
	 * A function that will zero out a geometry::Twist object, set the Twist object
	 * to represent moving forward in the x direction at rate of speed, for a certain
	 * amount of time in increments of dt. It is crucial that the rate of the
	 * ros::Rate object is 1.0 / dt.
	 */
	double move_forward_x(double speed, double dt, double total_time, geometry_msgs::Twist& cmd);

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

private:

	const double										_linear_vel;
	const double										_yaw_rate;
	const double										_sample_dt;
	const std::string									_pub_topic;
	ros::Publisher										_publisher;
	ros::Rate 											_timer;
	actionlib::SimpleActionServer<eecs376_ps4::PathAction>* 	_p_action_server;
	ros::Subscriber										_p_alarm_subscriber;
	bool 												_alarm_trigger;


};


#endif // end of EECS376_PS4_PATHEXECUTOR_H
