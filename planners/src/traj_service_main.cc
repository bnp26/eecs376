// SYSTEM INCLUDES

// ROS INCLUDES
#include <ros/ros.h>

// C++ PROJECT INCLUDES
#include "planners/pub_des_state.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "eecs_376_ps6_traj_service_node");
    ros::NodeHandle handle;

    DesStatePublisher pub(handle);

    ros::Rate looprate(1 / dt);
    pub.set_init_pose(-5, 0, 0); // x=0, y=0, phi=0

    while(ros::ok())
    {
        pub.pub_next_state();
        ros::spinOnce();
        looprate.sleep();
    }

    return 0;
}
