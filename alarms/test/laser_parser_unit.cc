// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// C++ PROJECT INCLUDES
#include "zeta_alarms/core/laser_parser.h"

typedef zeta::utils::Pair<unsigned int, unsigned int> index_pair_t;
typedef zeta::utils::Pair<double, double> theta_pair_t;
typedef zeta::utils::Pair<index_pair_t, theta_pair_t> params_t;

const double PI = 3.141592653589793238462643383279502884;
const double ROBOT_RADIUS = 0.5;
const double MIN_SAFE_DISTANCE = 1.0;

namespace zeta
{
namespace laser_alarm
{
namespace unit
{


TEST(zeta_laser_alarm_unit, test_get_scan_params_entire_scan)
{
    // set up scan
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI;
	laser_scan.angle_max = PI;
	laser_scan.angle_increment = PI / 12.0;
	laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);

    //ros::NodeHandle handle;
    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

	params_t params = parser.get_scan_params(laser_scan);
	EXPECT_EQ(params.first.first, 0);
	EXPECT_EQ(params.first.second, laser_scan.ranges.size());
	EXPECT_EQ(params.second.first, laser_scan.angle_min);
	EXPECT_EQ(params.second.second, laser_scan.angle_max);
}

TEST(zeta_laser_alarm_unit, test_get_scan_params_part_of_scan_to_end)
{
	// set up scan
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI - PI / 12.0;
	// std::cout << "angle_min" << laser_scan.angle_min << std::endl;
	laser_scan.angle_max = PI;
	laser_scan.angle_increment = PI / 12.0;
	laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);

    //ros::NodeHandle handle;
    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          -PI, laser_scan.angle_max, MIN_SAFE_DISTANCE);

	params_t params = parser.get_scan_params(laser_scan);
	// std::cout << "params: {{" << params.first.first << ", " << params.first.second << "}, {" << params.second.first << ", " << params.second.second << "}}" << std::endl;

	EXPECT_EQ(params.first.first, 1) << "index should start at 1";
	EXPECT_EQ(params.first.second, laser_scan.ranges.size()) << "last index should be end of scan";
	EXPECT_NEAR(params.second.first, -PI, 0.00009) << "theta_min should be -PI";
	EXPECT_NEAR(params.second.second, laser_scan.angle_max, 0.0009) << "theta_max should be PI";
}

TEST(zeta_laser_alarm_unit, test_get_scan_params_deeper_part_of_scan_to_end)
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

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          0.0, laser_scan.angle_max, MIN_SAFE_DISTANCE);

	params_t params = parser.get_scan_params(laser_scan);
	// std::cout << "params: {{" << params.first.first << ", " << params.first.second << "}, {" << params.second.first << ", " << params.second.second << "}}" << std::endl;

	EXPECT_EQ(params.first.first, laser_scan.ranges.size() / 2) << "index should start at halfway through the array";
	EXPECT_EQ(params.first.second, laser_scan.ranges.size()) << "last index should be end of scan";
	EXPECT_NEAR(params.second.first, 0.0, 0.00009) << "theta_min should be -PI";
	EXPECT_NEAR(params.second.second, laser_scan.angle_max, 0.0009) << "theta_max should be PI";
}

TEST(zeta_laser_alarm_unit, test_safe_to_move_no_obstacles)
{

	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    // std::cout << "number of measurements: " << laser_scan.ranges.size() << std::endl;
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = 3.0;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_TRUE(parser.safe_to_move(laser_scan, params));

}

TEST(zeta_laser_alarm_unit, test_safe_to_move_one_obstacle_distance_equals_max_distance)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = 1.0;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(parser.safe_to_move(laser_scan, params));
}

TEST(zeta_laser_alarm_unit, test_safe_to_move_one_obstacle_distance_less_than_max_distance)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = 0.5;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(parser.safe_to_move(laser_scan, params));
}

TEST(zeta_laser_alarm_unit, test_safe_to_move_one_obstacle_in_middle_of_scan)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (laser_scan.ranges.size() / 2 - 2 <= i && i <= laser_scan.ranges.size() / 2 + 2) ? 1.0 : 3.0;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(parser.safe_to_move(laser_scan, params));
}

TEST(zeta_laser_alarm_unit, test_safe_to_move_one_obstacle_along_right_corridor_wall)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    double theta_2 = asin(ROBOT_RADIUS / MIN_SAFE_DISTANCE);

    double theta = laser_scan.angle_min;
    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (theta > theta_2) ? 0.5 : 3.0;
    	theta += laser_scan.angle_increment;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(parser.safe_to_move(laser_scan, params));
}

TEST(zeta_laser_alarm_unit, test_safe_to_move_one_obstacle_along_left_corridor_wall)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    double theta_1 = -asin(ROBOT_RADIUS / MIN_SAFE_DISTANCE);

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    double theta = laser_scan.angle_min;
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (theta < theta_1) ? 0.5 : 3.0;
    	theta += laser_scan.angle_increment;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(parser.safe_to_move(laser_scan, params));
}

TEST(zeta_laser_alarm_unit, test_safe_to_move_obstacle_is_single_point)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (i == 9) ? 0.2 : 3.0;
    }

    zeta::laser_alarm::LaserParser parser(NULL, "pub", "sub", 1, ROBOT_RADIUS,
                                          laser_scan.angle_min, laser_scan.angle_max, MIN_SAFE_DISTANCE);

    params_t params = parser.get_scan_params(laser_scan);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(parser.safe_to_move(laser_scan, params));
}


} // end of namespace unit
} // end of namespace laser_alarm
} // end of namespace zeta
