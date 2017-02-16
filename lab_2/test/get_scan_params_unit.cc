// SYSTEM INCLUDES
#include <iostream>
#include <gtest/gtest.h>
#include <sensor_msgs/LaserScan.h>

// C++ PROJECT INCLUDES
#include "lab_2/lidar_parser.h"

typedef std::pair<unsigned int, unsigned int> index_pair;
typedef std::pair<double, double> theta_pair;

TEST(eecs376_ps2_unit, test_get_scan_params_entire_scan)
{
	// set up scan
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI;
	laser_scan.angle_max = PI;
	laser_scan.angle_increment = PI / 12.0;
	laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);

	std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
	EXPECT_EQ(params.first.first, 0);
	EXPECT_EQ(params.first.second, laser_scan.ranges.size());
	EXPECT_EQ(params.second.first, laser_scan.angle_min);
	EXPECT_EQ(params.second.second, laser_scan.angle_max);
}

TEST(eecs376_ps2_unit, test_get_scan_params_part_of_scan_to_end)
{
	// set up scan
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI - PI / 12.0;
	// std::cout << "angle_min" << laser_scan.angle_min << std::endl;
	laser_scan.angle_max = PI;
	laser_scan.angle_increment = PI / 12.0;
	laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);

	std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, -PI, laser_scan.angle_max);
	// std::cout << "params: {{" << params.first.first << ", " << params.first.second << "}, {" << params.second.first << ", " << params.second.second << "}}" << std::endl;

	EXPECT_EQ(params.first.first, 1) << "index should start at 1";
	EXPECT_EQ(params.first.second, laser_scan.ranges.size()) << "last index should be end of scan";
	EXPECT_NEAR(params.second.first, -PI, 0.00009) << "theta_min should be -PI";
	EXPECT_NEAR(params.second.second, laser_scan.angle_max, 0.0009) << "theta_max should be PI";
}

TEST(eecs376_ps2_unit, test_get_scan_params_deeper_part_of_scan_to_end)
{
	// set up scan
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI;
	// std::cout << "angle_min" << laser_scan.angle_min << std::endl;
	laser_scan.angle_max = PI;
	laser_scan.angle_increment = PI / 12.0;
	laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);

	/**
	std::cout << "{index, theta}: < ";
	for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
	{
	 	std::cout << "{" << i << "," << (laser_scan.angle_min + ((double)i) * laser_scan.angle_increment) << "}";
	 	if(i < laser_scan.ranges.size() - 1) { std::cout << ", "; }
	}
	std::cout << "> " << std::endl;
	*/

	// std::cout << "number of elements: " << laser_scan.ranges.size() << std::endl;

	std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, 0.0, laser_scan.angle_max);
	// std::cout << "params: {{" << params.first.first << ", " << params.first.second << "}, {" << params.second.first << ", " << params.second.second << "}}" << std::endl;

	EXPECT_EQ(params.first.first, laser_scan.ranges.size() / 2) << "index should start at halfway through the array";
	EXPECT_EQ(params.first.second, laser_scan.ranges.size()) << "last index should be end of scan";
	EXPECT_NEAR(params.second.first, 0.0, 0.00009) << "theta_min should be -PI";
	EXPECT_NEAR(params.second.second, laser_scan.angle_max, 0.0009) << "theta_max should be PI";
}
