# pragma once
#ifndef EECS376_PS6_TYPES_H
#define EECS376_PS6_TYPES_H

// SYSTEM INCLUDES


// ROS INCLUDES
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Twist.h>


// C++ PROJECT INCLUDES

typedef nav_msgs::Odometry odom_t;
typedef nav_msgs::Path path_t;
typedef geometry_msgs::Pose pose_t;
typedef geometry_msgs::PoseStamped pose_stamped_t;
typedef geometry_msgs::Quaternion quaternion_t;
typedef geometry_msgs::PoseWithCovariance pose_covar_t;
typedef geometry_msgs::Twist twist_t;

#endif // end of EECS376_PS6_TYPES_H
