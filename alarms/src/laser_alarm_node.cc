// SYSTEM INCLUDES
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

// C++ PROJECT INCLUDES
#include "alarms/core/laser_parser.h"

const double ROBOT_RADIUS = 0.5;
const double MIN_SAFE_DISTANCE = 0.3;
const double PI = 3.141592653589793238462643383279502884;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_alarm_node");
    ros::NodeHandle handle;

    const std::string pub_topic = "laser_alarm";
    const std::string sub_topic = "scan";
    const unsigned int sub_buffer_size = 1;

    // create a LaserParser object and give it a topic name
    zeta::laser_alarm::LaserParser laser_parser(&handle, pub_topic, sub_topic, sub_buffer_size,
                                                ROBOT_RADIUS, -PI/2, PI/2, MIN_SAFE_DISTANCE,
                                                true, "robot0");

    // custom version of ros::spin()
    laser_parser.execute();
    return 0;
}
