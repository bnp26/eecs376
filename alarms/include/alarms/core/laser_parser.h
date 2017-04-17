#pragma once
#ifndef ZETA_LASER_ALARM_LASER_PARSER_H
#define ZETA_LASER_ALARM_LASER_PARSER_H


// SYSTEM INCLUDES
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <utility>


// C++ PROJECT INCLUDES


namespace zeta
{
namespace laser_alarm
{

typedef std::pair<unsigned int, unsigned int> index_pair_t;
typedef std::pair<double, double> theta_pair_t;
typedef std::pair<index_pair_t, theta_pair_t> params_t;

class LaserParser
{
public:

    LaserParser(ros::NodeHandle* p_handle, const std::string& pub_topic, const std::string& sub_topic,
                const unsigned int sub_buffer_size, const double robot_radius, const double theta_min_radians,
                const double theta_max_radians, const double min_safe_distance,
                bool publish_field_of_view=false, const std::string& robot_frame_id="");

    virtual ~LaserParser();

    void construct_laser_alarm_view_field();

    void laser_callback(const sensor_msgs::LaserScan& laser_scan);

    const params_t get_scan_params(const sensor_msgs::LaserScan& laser_scan);

    const bool safe_to_move(const sensor_msgs::LaserScan& laser_scan, const params_t& params);

    void execute();

private:
    ros::NodeHandle*        _p_handle;
    ros::Publisher          _alarm_publisher, _field_of_view_publisher;
    ros::Subscriber         _subscriber;
    const double            _robot_radius, _theta_min_radians, _theta_max_radians, _min_safe_distance;
    sensor_msgs::LaserScan  _laser_alarm_view_field;
    const bool              _publish_field_of_view;
    const std::string       _frame_id;
};

} // end of namespace laser_alarm
} // end of namespace zeta

#endif // end of ZETA_LASER_ALARM_LASER_PARSER_H
