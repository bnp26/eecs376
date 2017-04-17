// SYSTEM INCLUDES
#include <std_msgs/Bool.h>

// C++ PROJECT INCLUDES
#include "alarms/core/laser_parser.h"


namespace zeta
{
namespace laser_alarm
{

LaserParser::LaserParser(ros::NodeHandle* p_handle, const std::string& pub_topic, const std::string& sub_topic,
                         const unsigned int sub_buffer_size, const double robot_radius, const double theta_min_radians,
                         const double theta_max_radians, const double min_safe_distance, bool publish_field_of_view,
                         const std::string& frame_id) : _p_handle(p_handle),
    _alarm_publisher(), _field_of_view_publisher(),
    _subscriber(),
    _robot_radius(robot_radius), _theta_min_radians(theta_min_radians), _theta_max_radians(theta_max_radians),
    _min_safe_distance(robot_radius + min_safe_distance), _laser_alarm_view_field(),
    _publish_field_of_view(publish_field_of_view), _frame_id(frame_id)
{
    if(this->_p_handle)
    {
        this->_alarm_publisher = p_handle->advertise<std_msgs::Bool>(pub_topic, sub_buffer_size);
        this->_subscriber = p_handle->subscribe(sub_topic, sub_buffer_size, &LaserParser::laser_callback, this);
    }
    if(this->_publish_field_of_view)
    {
        this->_field_of_view_publisher = p_handle->advertise<sensor_msgs::LaserScan>("laser_alarm_field_of_view_" + frame_id, 1);
        this->construct_laser_alarm_view_field();
        ROS_INFO("LaserParser instance created...you can now get the field of view by subscribing to topic: laser_alarm_field_of_view_%s", frame_id.c_str());
    }
}

LaserParser::~LaserParser()
{
}

void LaserParser::construct_laser_alarm_view_field()
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

    const double theta_2 = asin(this->_robot_radius / this->_min_safe_distance);
    const double theta_1 = -theta_2;  // make it symmetric about 0 radians
    double min_safe_distance = 0.0;

    this->_laser_alarm_view_field.header.frame_id = this->_frame_id;
    this->_laser_alarm_view_field.angle_min = this->_theta_min_radians;
    this->_laser_alarm_view_field.angle_increment = 0.01;

    this->_laser_alarm_view_field.range_min = 0.0;

    double theta = this->_laser_alarm_view_field.angle_min;
    const double angle_diff = fabs(this->_theta_max_radians - this->_laser_alarm_view_field.angle_min);
    const unsigned int num_elements = (unsigned int)(angle_diff / this->_laser_alarm_view_field.angle_increment);

    double max_element = 0.0;
    double element = 0.0;
    for(unsigned int i = 0; i < num_elements; ++i)
    {
        if (theta_1 <= theta && theta <= theta_2)
        {
            element = this->_min_safe_distance;
        }
        else
        {
            element = fabs(this->_robot_radius * (1.0 / sin(theta)));
        }

        if(element > max_element)
        {
            max_element = element;
        }
        this->_laser_alarm_view_field.ranges.push_back(element);
        this->_laser_alarm_view_field.intensities.push_back(1.0);

        theta += this->_laser_alarm_view_field.angle_increment;
    }
    this->_laser_alarm_view_field.range_max = max_element + 0.01;
    this->_laser_alarm_view_field.angle_max = theta;

    // we are going to construct the file of view just for publishing purposes (to view it)
    //
}

void LaserParser::laser_callback(const sensor_msgs::LaserScan& laser_scan)
{
    std_msgs::Bool alarm_msg;
    alarm_msg.data = !this->safe_to_move(laser_scan, this->get_scan_params(laser_scan));
    this->_alarm_publisher.publish(alarm_msg);
}

const params_t LaserParser::get_scan_params(const sensor_msgs::LaserScan& laser_scan)
{
    theta_pair_t thetas;
    index_pair_t indices;
    if(this->_theta_min_radians <= laser_scan.angle_min)
	{
		// we are asking for too wide a range...start from the beginning of the scan.
		thetas.first = laser_scan.angle_min;
		indices.first = 0;
	}
	else
	{
		// compute index and theta min
		indices.first = (int)abs((laser_scan.angle_min - this->_theta_min_radians) /
                                  laser_scan.angle_increment);
		thetas.first = laser_scan.angle_min + indices.first * laser_scan.angle_increment;
	}

	if(laser_scan.angle_max <= this->_theta_max_radians)
	{
		// we are asking for too wide a range...end with the end of the scan.
		thetas.second = laser_scan.angle_max;
		indices.second = laser_scan.ranges.size();
	}
	else
	{
		// compute index and theta max
		indices.second = (int)abs((laser_scan.angle_min - this->_theta_max_radians) /
                                  laser_scan.angle_increment);
		thetas.second = laser_scan.angle_min + indices.second * laser_scan.angle_increment;
	}
	return std::make_pair(indices, thetas);
}

const bool LaserParser::safe_to_move(const sensor_msgs::LaserScan& laser_scan, const params_t& params)
{

    double theta_2 = asin(this->_robot_radius / this->_min_safe_distance);
    double theta_1 = -theta_2;  // make it symmetric about 0 radians
    double min_safe_distance = 0.0;

    double theta = params.second.first;
    for(unsigned int index = params.first.first; index < params.first.second; ++index)
    {
    	// this is the piecewise part.
    	min_safe_distance = (theta_1 <= theta && theta <= theta_2) ? this->_min_safe_distance : this->_robot_radius * (1.0 / sin(theta));

    	// this is in place of the abs() function. I've found that abs() rounds values and I don't want that
    	// so I manually made it.
    	min_safe_distance = (min_safe_distance < 0) ? -1 * min_safe_distance : min_safe_distance;

    	// if the measurement is less than the safe distance, not safe to move forward so return false
    	if (fabs(laser_scan.ranges[index]) <= min_safe_distance)
    	{
    		return false;
    	}

    	// increment theta for the next measurement
    	theta += laser_scan.angle_increment;

    }

    // all measurements are safe, ok to move forward
    return true;
}

void LaserParser::execute()
{
    const double publish_rate_in_sec = 0.2;
    ros::Rate loop_rate(1.0 / publish_rate_in_sec);

    while(ros::ok())
    {
        this->_field_of_view_publisher.publish(this->_laser_alarm_view_field);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

} // end of namespace laser_alarm
} // end of namespace zeta
