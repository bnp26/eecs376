// SYSTEM INCLUDES
#include <gtest/gtest.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>

// C++ PROJECT INCLUDES
#include "lab2/lidar_parser.h"

typedef std::pair<unsigned int, unsigned int> index_pair;
typedef std::pair<double, double> theta_pair;

TEST(eecs376_ps2_unit, test_safe_to_move_no_obstacles)
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

    double robot_radius = 0.5;
    double max_distance = 1.0;

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_TRUE(safe_to_move(laser_scan, params, robot_radius, max_distance));

}

TEST(eecs376_ps2_unit, test_safe_to_move_one_obstacle_distance_equals_max_distance)
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

    double robot_radius = 0.5;
    double max_distance = 1.0;

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(safe_to_move(laser_scan, params, robot_radius, max_distance));
}

TEST(eecs376_ps2_unit, test_safe_to_move_one_obstacle_distance_less_than_max_distance)
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

    double robot_radius = 0.5;
    double max_distance = 1.0;

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(safe_to_move(laser_scan, params, robot_radius, max_distance));
}

TEST(eecs376_ps2_unit, test_safe_to_move_one_obstacle_in_middle_of_scan)
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

    double robot_radius = 0.5;
    double max_distance = 1.0;

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(safe_to_move(laser_scan, params, robot_radius, max_distance));
}

TEST(eecs376_ps2_unit, test_safe_to_move_one_obstacle_along_right_corridor_wall)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    double robot_radius = 0.5;
    double max_distance = 1.0;

    double theta_2 = asin(robot_radius / max_distance);

    double theta = laser_scan.angle_min;
    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (theta > theta_2) ? 0.5 : 3.0;
    	theta += laser_scan.angle_increment;
    }

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(safe_to_move(laser_scan, params, robot_radius, max_distance));
}

TEST(eecs376_ps2_unit, test_safe_to_move_one_obstacle_along_left_corridor_wall)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    double robot_radius = 0.5;
    double max_distance = 1.0;

    double theta_1 = -asin(robot_radius / max_distance);

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    double theta = laser_scan.angle_min;
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (theta < theta_1) ? 0.5 : 3.0;
    	theta += laser_scan.angle_increment;
    }

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(safe_to_move(laser_scan, params, robot_radius, max_distance));
}

TEST(eecs376_ps2_unit, test_safe_to_move_obstacle_is_single_point)
{
	sensor_msgs::LaserScan laser_scan;
	laser_scan.angle_min = -PI / 2.0;
    laser_scan.angle_max = PI / 2.0;
    laser_scan.angle_increment = PI / 12.0;
    laser_scan.range_min = 0.0;
    laser_scan.range_max = 3.0;

    double robot_radius = 0.5;
    double max_distance = 1.0;

    laser_scan.ranges.resize((int)((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1);
    for(unsigned int i = 0; i < laser_scan.ranges.size(); ++i)
    {
    	laser_scan.ranges[i] = (i == 9) ? 0.2 : 3.0;
    }

    std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, laser_scan.angle_min, laser_scan.angle_max);
    EXPECT_EQ(params.first.first, 0);
    EXPECT_EQ(params.first.second, laser_scan.ranges.size());

    EXPECT_FALSE(safe_to_move(laser_scan, params, robot_radius, max_distance));
}
