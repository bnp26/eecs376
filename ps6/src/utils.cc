// SYSTEM INCLUDES


// ROS INCLUDES


// C++ PROEJCT INCLUDES
#include "eecs376_ps6/utils.h"

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

const double min_dang(const double dang)
{
    double modified_dang = dang;
    while(modified_dang > PI) { modified_dang -= 2.0 * PI; }
    while(modified_dang < PI) { modified_dang += 2.0 * PI; }
    return modified_dang;
}

const double sat(const double x)
{
    if (x > 1.0) { return 1.0; }
    if (x < -1.0) { return -1.0; }
    return x;
}

const double sign(const double x)
{
    if (x > 0.0) { return 1.0; }
    if (x < 0.0) { return -1.0; }
    return 0.0;
}

const double clamp(const double min, const double val, const double max)
{
    if(val <= min)
    {
        return min;
    }
    else if(max <= val)
    {
        return max;
    }
    else
    {
        return val;
    }
}

const double convert_planar_quaternion_to_phi(const quaternion_t& q)
{
    return 2.0 * atan2(q.z, q.w);
}

const quaternion_t convert_planar_phi_to_quaternion(const double phi)
{
    quaternion_t q;
    zero_quaternion(q);
    q.z = sin(phi / 2.0);
    q.w = cos(phi / 2.0);
    return q;
}

pose_stamped_t xyphi_to_pose_stamped(const double x, const double y, const double phi)
{
    pose_stamped_t p_stamped;
    p_stamped.pose.orientation = convert_planar_phi_to_quaternion(phi);
    p_stamped.pose.position.x = x;
    p_stamped.pose.position.y = y;
    p_stamped.pose.position.z = 0.0;
    return p_stamped;
}

pose_t computed_curved_pose(const odom_t& prior_state, const double dt)
{
    pose_t pose;

    const double prior_x_position = prior_state.pose.pose.position.x;
    const double prior_y_position = prior_state.pose.pose.position.y;

    const double prior_angle = convert_planar_quaternion_to_phi(prior_state.pose.pose.orientation);

    const double current_linear_vel = prior_state.twist.twist.linear.x;
    const double current_angular_vel = prior_state.twist.twist.angular.z;

    pose.position.x = prior_x_position + current_linear_vel * (cos(current_angular_vel * dt) - dt);
    pose.position.y = prior_y_position + current_linear_vel * sin(current_angular_vel * dt);
    pose.orientation = convert_planar_phi_to_quaternion(prior_angle + current_angular_vel * dt);
    return pose;
}
