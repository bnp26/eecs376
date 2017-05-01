// SYSTEM INCLUDES
#include <ros/ros.h>
#include <std_msgs/Bool.h>

// C++ PROJECT INCLUDES


int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_gripper_client_test");
	ros::NodeHandle nh;
	
	ros::Publisher gripper_pub = nh.advertise<std_msgs::Bool>("close_gripper", 1, true);
	
	// close the gripper
	std_msgs::Bool msg;
	msg.data = true;
	gripper_pub.publish(msg);
	
	ROS_INFO("Waiting 2 seconds for gripper to actually close");
	ros::Duration(2.0).sleep();
	
	// open the gripper
	msg.data = false;
	gripper_pub.publish(msg);
}
