// SYSTEM INCLUDES
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// C++ PROJECT INCLUDES


int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_open_gripper_client");
	ros::NodeHandle nh;
	ros::Publisher gripper = nh.advertise<std_msgs::Bool>("close_gripper", 1);

    std_msgs::Bool grab;
    //grab

    grab.data = false;
    ros::Time start =  ros::Time::now();  
    while((ros::Time::now() - start) < ros::Duration(3)) {
		ROS_INFO("PUBLISHING GRAB=FALSE");
        gripper.publish(grab);
        ros::Duration(0.3).sleep();
        ros::spinOnce();
    }
}
