// SYSTEM INCLUDES
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>


// C++ PROJECT INCLUDES
#include "eecs376_ps2/lidar_parser.h"

ros::Publisher lidar_alarm_pub;

// save some keystrokes
typedef std::pair<unsigned int, unsigned int> index_pair;
typedef std::pair<double, double> theta_pair;


// Callback for checking if there is an object in the current laser scan.
// This will be called every time this node receives a sensor_msgs::LaserScan message.
void laser_callback(const sensor_msgs::LaserScan& laser_scan)
{
	const double MIN_SAFE_DISTANCE = 1.0;
	const double ROBOT_RADIUS = 0.5;

	std_msgs::Bool alarm_msg;
	std::pair<index_pair, theta_pair> params = get_scan_params(laser_scan, -PI/2.0, PI/2.0);

	// IMPORTANT! need to invert result of safe_to_move() because it returns true if there
	// is no alarm and false otherwise, which is the opposite of how this message is used.
	alarm_msg.data = !safe_to_move(laser_scan, params, ROBOT_RADIUS, MIN_SAFE_DISTANCE);
	lidar_alarm_pub.publish(alarm_msg);
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "lidar_alarm");
  	ros::NodeHandle n_handle; 

    //create a Subscriber object and have it subscribe to the lidar topic
    lidar_alarm_pub = n_handle.advertise<std_msgs::Bool>("lidar_alarm", 1);
    ros::Subscriber lidar_subscriber = n_handle.subscribe("robot0/laser_0", 1, laser_callback);

    ros::spin();

    return 0;
}
