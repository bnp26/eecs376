// SYSTEM INCLUDES


// ROS INCLUDES


// C++ PROEJCT INCLUDES
#include "eecs376_ps6/Utils.h"

twist_t& zero_twist_cmd(twist_t& cmd)
{
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
}

quaternion_t& zero_quaternion(quaternion_t& q)
{
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
    return q;
}
