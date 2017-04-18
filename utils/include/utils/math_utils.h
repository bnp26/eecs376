#pragma once
#ifndef UTILS_MATH_UTILS_MATH_UTILS_H
#define UTILS_MATH_UTILS_MATH_UTILS_H

// SYSTEM INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>


// C++ PROJECT INCLUDES


namespace math_utils
{

double clamp(const double min, const double val, const double max);

double sign(const double val);

double min_angle(const double angle);

double convert_quaternion_to_angle(const geometry_msgs::Quaternion& q);

geometry_msgs::Quaternion convert_angle_to_quaternion(const double psi);

} // end of namespace math_utils

#endif // end of UTILS_MATH_UTILS_MATH_UTILS_H

