// SYSTEM INCLUDES
#include <ros/ros.h>
#include <math.h>
// #include <iostream>

// C++ PROJECT INCLUDES
#include "lab_2/lidar_parser.h"

const std::pair<index_pair, theta_pair> get_scan_params(const sensor_msgs::LaserScan& laser_scan,
														const double theta_min, const double theta_max)
{
	theta_pair thetas;
	index_pair indices;
	if(theta_min <= laser_scan.angle_min)
	{
		// we are asking for too wide a range...start from the beginning of the scan.
		thetas.first = laser_scan.angle_min;
		indices.first = 0;
	}
	else
	{
		// compute index and theta min
		indices.first = (int)abs((laser_scan.angle_min - theta_min) / laser_scan.angle_increment);
		thetas.first = laser_scan.angle_min + indices.first * laser_scan.angle_increment;
	}

	if(laser_scan.angle_max <= theta_max)
	{
		// we are asking for too wide a range...end with the end of the scan.
		thetas.second = laser_scan.angle_max;
		indices.second = laser_scan.ranges.size();
	}
	else
	{
		// compute index and theta max
		indices.second = (int)abs((laser_scan.angle_min - theta_max) / laser_scan.angle_increment);
		thetas.second = laser_scan.angle_min + indices.second * laser_scan.angle_increment;
	}
	return std::make_pair(indices, thetas);
}


bool safe_to_move(const sensor_msgs::LaserScan& laser_scan, const std::pair<index_pair, theta_pair>& params,
				  const float robot_radius, const float max_distance)
{
	// the strategy is to draw a "corridor" around the robot.

	// We also want to only look for objects that are a certain radius away from the robot.
	// Therefore, the corresponding "shape" that we will look for objects within is a rectangle
	// with a curved end. If a lidar scan shows an object within this shape, the robot cannot forwards,
	// and needs to turn.

	// The shape looks like (the robot is the '0' at the left end):
	//  __________
	// 0__________)
	//

	// The width of this corridor is the diameter of the robot, and the curved end of the corridor is the max radius
	// that the robot can come to objects before turning.

	// To check for objects within this shape, we will use a piecewise function. When the value of a lidar scan for a particular
	// theta value touches the corridor, we will check if the distance is less than the expected distance of the corridor, otherwise
	// the distance should be less than the maximum radius.

	// Here is the corridor again with significant theta values:
	//			theta_1
	//  __________|
	// 0__________)
	//			  |
	//			theta_2
	//
	// for a measurement y_i: if y_i is less than the expected distance "d_i," then there is an object and the robot must turn.
	// 						[ max_distance if theta_2 <= theta <= theta_1
	// d_i is defined as: 	[
	//						[ r*csc(theta) otherwise

	// r*sec(theta) is derived from makin a right triangle from the center of the robot to the corridor touching the robot and out
	// to a point along the corridor. Therefore:
	//		r / distance_along_corridor = cot(theta)
	// and the hypotenuse of the triangle = Sqrt(r^2 + distance_along_corridor^2),
	// then the hypotenuse = r*csc(theta)
	//
	// The hypotenuse is returned by the lidar scan, so if the measured distance is <= r*sec(theta) then the robot will not be able
	// to move forward because of an obstacle.

	// theta_1 is computed by arcsin(r / max_distance), and theta_2 = 2 * (Pi/2 - theta_1) + theta_1 = Pi - theta_1

	double theta_2 = asin(robot_radius / max_distance);
	double theta_1 = -theta_2;  // make it symmetric about 0 radians
    double min_safe_distance = 0.0;

    double theta = params.second.first;
    for(unsigned int index = params.first.first; index < params.first.second; ++index)
    {
    	// this is the piecewise part.
    	min_safe_distance = (theta_1 <= theta && theta <= theta_2) ? max_distance : robot_radius * (1.0 / sin(theta));

    	// this is in place of the abs() function. I've found that abs() rounds values and I don't want that
    	// so I manually made it.
    	min_safe_distance = (min_safe_distance < 0) ? -1 * min_safe_distance : min_safe_distance;

    	// if the measurement is less than the safe distance, not safe to move forward so return false
    	if (laser_scan.ranges[index] <= min_safe_distance)
    	{
    		return false;
    	}

    	// increment theta for the next measurement
    	theta += laser_scan.angle_increment;

    }

    // all measurements are safe, ok to move forward
    return true;

}
