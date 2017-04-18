// SYSTEM INCLUDES


// ROS INCLUDES
#include <ros/ros.h>


// C++ PROJECT INCLUDES
#include "planners/trajectory_builder.h"
#include "planners/utils.h"
#include "planners/constants.h"


TrajBuilder::TrajBuilder() : _dt(default_dt), _a_max(default_a_max), _v_max(default_v_max),
    _alpha_max(default_alpha_max), _omega_max(default_omega_max), _path_move_tol(default_path_move_tol),
    _zero_twist()
{
    zero_twist_cmd(this->_zero_twist);
}

TrajBuilder::~TrajBuilder()
{
}

void TrajBuilder::set_dt(const double dt)
{
    ROS_INFO("setting dt to %f", dt);
    this->_dt = dt;
}

void TrajBuilder::set_a_max(const double a_max)
{
    this->_a_max = a_max;
}

void TrajBuilder::set_v_max(const double v_max)
{
    this->_v_max = v_max;
}

void TrajBuilder::set_alpha_max(const double alpha_max)
{
    this->_alpha_max = alpha_max;
}

void TrajBuilder::set_omega_max(const double omega_max)
{
    this->_omega_max = omega_max;
}

void TrajBuilder::set_path_move_tol(const double path_move_tol)
{
    this->_path_move_tol = path_move_tol;
}

//here are the main traj-builder fncs:
void TrajBuilder::build_trapezoidal_spin_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                              std::vector<odom_t>& vec_of_states)
{
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convert_planar_quaternion_to_phi(start_pose.pose.orientation);
    double psi_end = convert_planar_quaternion_to_phi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    double t_ramp = this->_omega_max/ this->_alpha_max;
    double ramp_up_dist = 0.5 * this->_alpha_max * t_ramp * t_ramp;
    double cruise_distance = fabs(dpsi) - 2.0 * ramp_up_dist; //delta-angle to spin at omega_max
    int npts_ramp = round(t_ramp / this->_dt);
    odom_t des_state;
    des_state.header.frame_id = start_pose.header.frame_id; //really, want to copy the frame_id
    des_state.header.stamp = ros::Time::now();
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest

    //ramp up omega (positive or negative);
    double t = 0.0;
    double accel = sign(dpsi) * this->_alpha_max;
    double omega_des = 0.0;
    double psi_des = psi_start;
    for (int i = 0; i < npts_ramp; i++) {
        t += this->_dt;
        omega_des = accel * t;
        des_state.twist.twist.angular.z = omega_des; //update rotation rate
        //update orientation
        psi_des = psi_start + 0.5 * accel * t * t;
        des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //now cruise for distance cruise_distance at const omega
    omega_des = sign(dpsi) * this->_omega_max;
    des_state.twist.twist.angular.z  = sign(dpsi) * this->_omega_max;
    double t_cruise = cruise_distance / this->_omega_max;
    int npts_cruise = round(t_cruise / this->_dt);
    for (int i = 0; i < npts_cruise; i++) {
        //Euler one-step integration
        psi_des += omega_des * this->_dt; //Euler one-step integration
        des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //ramp down omega to halt:
    for (int i = 0; i < npts_ramp; i++) {
        omega_des -= accel * this->_dt; //Euler one-step integration
        des_state.twist.twist.angular.z = omega_des;
        psi_des += omega_des * this->_dt; //Euler one-step integration
        des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where desired, and at rest:
    des_state.pose.pose = end_pose.pose; //
    des_state.twist.twist = this->_zero_twist; // insist on full stop
    vec_of_states.push_back(des_state);
}

void TrajBuilder::build_trapezoidal_travel_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                                std::vector<odom_t>& vec_of_states)
{
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = this->_v_max / this->_a_max;
    double ramp_up_dist = 0.5 * this->_a_max * t_ramp*t_ramp;
    double cruise_distance = trip_len - 2.0 * ramp_up_dist; //distance to travel at v_max
    ROS_INFO("t_ramp =%f",t_ramp);
    ROS_INFO("ramp-up dist = %f",ramp_up_dist);
    ROS_INFO("cruise distance = %f",cruise_distance);
    //start ramping up:
    odom_t des_state;
    des_state.header.frame_id = start_pose.header.frame_id; //really, want to copy the frame_id
    des_state.header.stamp = ros::Time::now();
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest
    int npts_ramp = round(t_ramp / this->_dt);
    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist

    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += this->_dt;
        speed_des = this->_a_max*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * this->_a_max * t * t * cos(psi_des);
        y_des = y_start + 0.5 * this->_a_max * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //now cruise for distance cruise_distance at const speed
    speed_des = this->_v_max;
    des_state.twist.twist.linear.x = speed_des;
    double t_cruise = cruise_distance / this->_v_max;
    int npts_cruise = round(t_cruise / this->_dt);
    ROS_INFO("t_cruise = %f; npts_cruise = %d",t_cruise,npts_cruise);
    for (int i = 0; i < npts_cruise; i++) {
        //Euler one-step integration
        x_des += speed_des * this->_dt * cos(psi_des);
        y_des += speed_des * this->_dt * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= this->_a_max*this->_dt; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * this->_dt * cos(psi_des); //Euler one-step integration
        y_des += speed_des * this->_dt * sin(psi_des); //Euler one-step integration
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

void TrajBuilder::build_triangular_spin_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                             std::vector<odom_t>& vec_of_states)
{
    odom_t des_state;
    des_state.header.frame_id = start_pose.header.frame_id; //really, want to copy the frame_id
    des_state.header.stamp = ros::Time::now();
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest
    vec_of_states.push_back(des_state);
    double psi_start = convert_planar_quaternion_to_phi(start_pose.pose.orientation);
    double psi_end = convert_planar_quaternion_to_phi(end_pose.pose.orientation);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO("spin traj: psi_start = %f; psi_end = %f; dpsi= %f", psi_start, psi_end, dpsi);
    double t_ramp = sqrt(fabs(dpsi) / this->_alpha_max);
    int npts_ramp = round(t_ramp / this->_dt);
    double psi_des = psi_start; //start from here
    double omega_des = 0.0; // assumes spin starts from rest;
    // position of des_state will not change; only orientation and twist
    double t = 0.0;
    double accel = sign(dpsi) * this->_alpha_max; //watch out for sign: CW vs CCW rotation
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += this->_dt;
        omega_des = accel*t;
        des_state.twist.twist.angular.z = omega_des; //update rotation rate
        //update orientation
        psi_des = psi_start + 0.5 * accel * t*t;
        des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        omega_des -= accel*this->_dt; //Euler one-step integration
        des_state.twist.twist.angular.z = omega_des;
        psi_des += omega_des*this->_dt; //Euler one-step integration
        des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose; //start from here
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

void TrajBuilder::build_triangular_travel_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                               std::vector<odom_t>& vec_of_states)
{
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_des = atan2(dy, dx);
    odom_t des_state;
    des_state.header.frame_id = start_pose.header.frame_id; //really, want to copy the frame_id
    des_state.header.stamp = ros::Time::now();
    des_state.pose.pose = start_pose.pose; //start from here
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest
    double trip_len = sqrt(dx * dx + dy * dy);
    double t_ramp = sqrt(trip_len / this->_a_max);
    int npts_ramp = round(t_ramp / this->_dt);
    double v_peak = this->_a_max*t_ramp; // could consider special cases for reverse motion
    double d_vel = this->_alpha_max*this->_dt; // incremental velocity changes for ramp-up

    double x_des = x_start; //start from here
    double y_des = y_start;
    double speed_des = 0.0;
    des_state.twist.twist.angular.z = 0.0; //omega_des; will not change
    des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des); //constant
    // orientation of des_state will not change; only position and twist
    double t = 0.0;
    //ramp up;
    for (int i = 0; i < npts_ramp; i++) {
        t += this->_dt;
        speed_des = this->_a_max*t;
        des_state.twist.twist.linear.x = speed_des; //update speed
        //update positions
        x_des = x_start + 0.5 * this->_a_max * t * t * cos(psi_des);
        y_des = y_start + 0.5 * this->_a_max * t * t * sin(psi_des);
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //ramp down:
    for (int i = 0; i < npts_ramp; i++) {
        speed_des -= this->_a_max*this->_dt; //Euler one-step integration
        des_state.twist.twist.linear.x = speed_des;
        x_des += speed_des * this->_dt * cos(psi_des); //Euler one-step integration
        y_des += speed_des * this->_dt * sin(psi_des); //Euler one-step integration
        des_state.pose.pose.position.x = x_des;
        des_state.pose.pose.position.y = y_des;
        vec_of_states.push_back(des_state);
    }
    //make sure the last state is precisely where requested, and at rest:
    des_state.pose.pose = end_pose.pose;
    //but final orientation will follow from point-and-go direction
    des_state.pose.pose.orientation = convert_planar_phi_to_quaternion(psi_des);
    des_state.twist.twist = this->_zero_twist; // insist on starting from rest
    vec_of_states.push_back(des_state);
}

void TrajBuilder::build_spin_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                  std::vector<odom_t>& vec_of_states)
{
    //decide if triangular or trapezoidal profile:
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double psi_start = convert_planar_quaternion_to_phi(start_pose.pose.orientation);
    double psi_end = convert_planar_quaternion_to_phi(end_pose.pose.orientation);
    ROS_INFO("current angle: %f, goal angle: %f", psi_start, psi_end);
    double dpsi = min_dang(psi_end - psi_start);
    ROS_INFO("rotational spin distance = %f", dpsi);
    double ramp_up_time = this->_omega_max/ this->_alpha_max;
    double ramp_up_dist = 0.5 * this->_alpha_max * ramp_up_time*ramp_up_time;
    //decide on triangular vs trapezoidal:
    if (fabs(dpsi) < 2.0 * ramp_up_dist) { //delta-angle is too short for trapezoid
        this->build_triangular_spin_traj(start_pose, end_pose, vec_of_states);
    } else {
        this->build_trapezoidal_spin_traj(start_pose, end_pose, vec_of_states);
    }
}

void TrajBuilder::build_travel_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                    std::vector<odom_t>& vec_of_states)
{
    //decide if triangular or trapezoidal profile:
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double trip_len = sqrt(dx * dx + dy * dy);
    double ramp_up_dist = 0.5 * this->_v_max * this->_v_max / this->_alpha_max;
    ROS_INFO("trip len = %f", trip_len);
    if (trip_len < 2.0 * ramp_up_dist) { //length is too short for trapezoid
        this->build_triangular_travel_traj(start_pose, end_pose, vec_of_states);
    } else {
        this->build_trapezoidal_travel_traj(start_pose, end_pose, vec_of_states);
    }
}

void TrajBuilder::build_point_and_go_traj(const pose_stamped_t start_pose, const pose_stamped_t end_pose,
                                          std::vector<odom_t>& vec_of_states)
{
    ROS_INFO("building point-and-go trajectory");
    odom_t bridge_state;
    pose_stamped_t bridge_pose; //bridge end of prev traj to start of new traj
    vec_of_states.clear(); //get ready to build a new trajectory of desired states
    ROS_INFO("building rotational trajectory");
    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = end_pose.pose.position.x;
    double y_end = end_pose.pose.position.y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double des_psi = atan2(dy, dx); //heading to point towards goal pose
    ROS_INFO("desired heading to subgoal = %f", des_psi);
    //bridge pose: state of robot with start_x, start_y, but pointing at next subgoal
    //  achieve this pose with a spin move before proceeding to subgoal with translational
    //  motion
    bridge_pose = start_pose;
    bridge_pose.pose.orientation = convert_planar_phi_to_quaternion(des_psi);
    ROS_INFO("building reorientation trajectory");
    this->build_spin_traj(start_pose, bridge_pose, vec_of_states); //build trajectory to reorient
    //start next segment where previous segment left off
    ROS_INFO("building translational trajectory");
    this->build_travel_traj(bridge_pose, end_pose, vec_of_states);
}

void TrajBuilder::build_braking_traj(const pose_stamped_t current_pose, const odom_t current_state, std::vector<odom_t>& vec_of_states)
{
	vec_of_states.clear();

    double initial_angular_velocity = current_state.twist.twist.angular.z;
    double initial_linear_velocity = current_state.twist.twist.linear.x;

    odom_t new_desired_state;
    new_desired_state.header = current_pose.header;
    new_desired_state.pose.pose = current_pose.pose;
    new_desired_state.twist = current_state.twist;

    // stop all motion
    double& current_angular_vel = new_desired_state.twist.twist.angular.z;
    double& current_linear_vel = new_desired_state.twist.twist.linear.x;

    const double signed_a_max = sign(current_linear_vel) * this->_a_max;
    const double signed_alpha_max = sign(current_angular_vel) * this->_alpha_max;

    const double min_linear_vel = fmin(0.0, initial_linear_velocity);
    const double max_linear_vel = fmax(0.0, initial_linear_velocity);

    double min_angular_vel = fmin(0.0, initial_angular_velocity);
    double max_angular_vel = fmax(0.0, initial_angular_velocity);

    const double ramp_down_linear_time = current_linear_vel / this->_a_max;
    const double ramp_down_angular_time = current_angular_vel / this->_alpha_max;

    const unsigned int num_ramp_down_points = round(fmax(ramp_down_linear_time, ramp_down_angular_time) / this->_dt);

    for(unsigned int i = 0; i < num_ramp_down_points; ++i)
    {
        new_desired_state.pose.pose = computed_curved_pose(new_desired_state, this->_dt);
        current_linear_vel -= clamp(min_linear_vel, current_linear_vel, max_linear_vel) * this->_dt;
        current_angular_vel -= clamp(min_angular_vel, current_angular_vel, max_angular_vel) * this->_dt;
        vec_of_states.push_back(new_desired_state);
    }
    new_desired_state.twist.twist = this->_zero_twist;
    vec_of_states.push_back(new_desired_state);

}
