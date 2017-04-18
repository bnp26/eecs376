#pragma once
#ifndef UTILS_MESSAGE_UTILS_MESSAGE_UTILS_H
#define UTILS_MESSAGE_UTILS_MESSAGE_UTILS_H


// SYSTEM INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


// C++ PROJECT INCLUDES


namespace message_utils
{

geometry_msgs::Quaternion create_zero_quaternion();

geometry_msgs::Pose create_zero_pose();

geometry_msgs::PoseStamped create_zero_pose_stamped();

} // end of namespace message_utils

#endif // end of UTILS_MESSAGE_UTILS_MESSAGE_UTILS_H

