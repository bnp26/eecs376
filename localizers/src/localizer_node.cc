// SYSTEM INCLUDES
#include <ros/ros.h>

// C++ PROJECT INCLUDER
#include "localizers/odom_tf.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer_node");
    ros::NodeHandle handle;
    OdomTf localizer(&handle);
    ros::spin();
}
