#pragma once
#ifndef EECS376_PS6_UTILS_H
#define EECS376_PS6_UTILS_H

// SYSTEM INCLUDES


// ROS INCLUDES


// C++ PROJECT INCLUDES
#include "eecs376_ps6/types.h"

twist_t& zero_twist_cmd(twist_t& cmd);

quaternion_t& zero_quaternion(quaternion_t& q);

#endif // end of EECS376_PS6_UTILS_H
