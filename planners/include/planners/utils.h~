#pragma once
#ifndef EECS376_PS6_UTILS_H
#define EECS376_PS6_UTILS_H

// SYSTEM INCLUDES


// ROS INCLUDES


// C++ PROJECT INCLUDES
#include "eecs376_ps6/types.h"

twist_t& zero_twist_cmd(twist_t& cmd);

quaternion_t& zero_quaternion(quaternion_t& q);

const double min_dang(const double dang);

const double sat(const double x);

const double sign(const double x);

const double clamp(const double min, const double val, const double max);

const double convert_planar_quaternion_to_phi(const quaternion_t& q);

const quaternion_t convert_planar_phi_to_quaternion(const double phi);

pose_stamped_t xyphi_to_pose_stamped(const double x, const double y, const double phi);

pose_t computed_curved_pose(const odom_t& prior_state, const double dt);

#endif // end of EECS376_PS6_UTILS_H
