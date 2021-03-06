#pragma once
#ifndef ZETA_PLANNERS_TRAJ_BUILDER_H
#define ZETA_PLANNERS_TRAJ_BUILDER_H

// SYSTEM INCLUDES
#include <queue>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

// ROS INCLUDES

// C++ PROJECT INCLUDES
#include "planners/types.h"


const double default_a_max = 0.5; //1m/sec^2
const double default_alpha_max = 0.4; //1 rad/sec^2
const double default_v_max = 1.5; //1 m/sec
const double default_omega_max = 0.05; //1 rad/sec
const double default_path_move_tol = 0.01; // if path points are within 1cm, fuggidaboutit
const double default_dt=0.0095;


class TrajBuilder
{
public:

    TrajBuilder();

    virtual ~TrajBuilder();

    void set_dt(const double dt);

    void set_a_max(const double a_max);

    void set_v_max(const double v_max);

    void set_alpha_max(const double alpha_max);

    void set_omega_max(const double omega_max);

    void set_path_move_tol(const double tol);

    //here are the main traj-builder fncs:
    void build_trapezoidal_spin_traj(const pose_stamped_t start_pose,
                                     const pose_stamped_t end_pose,
                                     std::vector<odom_t>& vec_of_states);

    void build_triangular_spin_traj(const pose_stamped_t start_pose,
                                    const pose_stamped_t end_pose,
                                    std::vector<odom_t>& vec_of_states);

    void build_spin_traj(const pose_stamped_t start_pose,
                         const pose_stamped_t end_pose,
                         std::vector<odom_t>& vec_of_states);

    void build_travel_traj(const pose_stamped_t start_pose,
                           const pose_stamped_t end_pose,
                           std::vector<odom_t>& vec_of_states);

    void build_trapezoidal_travel_traj(const pose_stamped_t start_pose,
                                       const pose_stamped_t end_pose,
                                       std::vector<odom_t>& vec_of_states);

    void build_triangular_travel_traj(const pose_stamped_t start_pose,
                                      const pose_stamped_t end_pose,
                                      std::vector<odom_t>& vec_of_states);

    void build_point_and_go_traj(const pose_stamped_t start_pose,
                                 const pose_stamped_t end_pose,
                                 std::vector<odom_t>& vec_of_states);

    void build_braking_traj(const pose_stamped_t current_pose, const odom_t current_state,
                            std::vector<odom_t>& vec_of_states);

    void build_backup_triangular_travel_traj(const pose_stamped_t current_pose,
                                             const pose_stamped_t end_pose,
                                             std::vector<odom_t>& vec_of_states);

    void build_backup_trapezoidal_travel_traj(const pose_stamped_t current_pose,
                                              const pose_stamped_t end_pose,
                                              std::vector<odom_t>& vec_of_states);

    void build_backup_traj(const pose_stamped_t current_pose,
                           const pose_stamped_t end_pose,
                           std::vector<odom_t>& vec_of_states);


protected:
private:

    double _dt;
    double _a_max;
    double _v_max;
    double _alpha_max;
    double _omega_max;
    double _path_move_tol;

    twist_t _zero_twist;

};

#endif // end of ZETA_PLANNERS_TRAJ_BUILDER_H
