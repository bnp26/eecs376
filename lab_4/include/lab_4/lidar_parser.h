#pragma once
#ifndef LAB_4_LIDAR_PARSER_H
#define LAB_4_LIDAR_PARSER_H

// SYSTEM INCLUDES
#include <sensor_msgs/LaserScan.h>
#include <utility>

// C++ PROJECT INCLUDES


#define PI 3.14159265

// save some keystrokes
typedef std::pair<unsigned int, unsigned int> index_pair;
typedef std::pair<double, double> theta_pair;


// get the parameters for a scan. This allows for only analyzing part of a scan depending
// on if the laser scan contains a wider angle range than theta_min -> theta_max.
// This method returns two things:
//		1) The index of the interval in the laser scan which corresponds to the theta range asked for.
//		2) The theta range that is actually contained in the scan. This is necessary for two reasons:
//			a) The theta range asked for may be wider than the range contained in the laser scan.
//			   In this case the returned theta range is the range of the scan.
//			b) The laser scan may not contain EXACTLY the range asked for...as in the theta values
//			   contained in the scan are not EXACTLY from theta_min -> theta_max.
//			   In this case the returned theta range is the closes theta values where theta_1 <= a <= b <= theta_max.
const std::pair<index_pair, theta_pair> get_scan_params(const sensor_msgs::LaserScan& laser_scan,
														const double theta_min, const double theta_max);


// This method determines if an obstacle exists in the laser scan interval specified by params,
// and is close enough to the robot determined by max_radius for a circular robot of radius robot_radius.
// This method returns true if it is safe to move (forwared) and false otherwise.
bool safe_to_move(const sensor_msgs::LaserScan& laser_scan, const std::pair<index_pair, theta_pair>& params,
				  const float robot_radius, const float max_radius);

#endif // end of LAB_4_LIDAR_PARSER_H
