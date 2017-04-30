//pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service

#include <ros/ros.h>
#include <hw_msgs/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "append_path_client");
    ros::NodeHandle n;
    ros::ServiceClient forward_client = n.serviceClient<hw_msgs::path>("append_path_queue_service");
    ros::ServiceClient flush_client = n.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
    geometry_msgs::Quaternion quat;
    
    while (!forward_client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    hw_msgs::path path;
    
    
    ROS_INFO("Step 1) navigate to the stool");
    // 1) NAVIGATE TO THE STOOL
    quat = convertPlanarPhi2Quaternion(0);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation = quat;
    
    pose_stamped.pose.position.x = 4.0;
    pose_stamped.pose.position.y = 0;
    path.request.path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x = 4.0;
    pose_stamped.pose.position.y = 2.6; 
    path.request.path.poses.push_back(pose_stamped);
    forward_client.call(path);
    
    // WAIT FOR THE ROBOT TO GET THERE
    ros::Duration(45.0).sleep();
    
    
    ROS_INFO("Step 2) find the stool and the block");
    // 2) FIND THE STOOL AND BLOCK
    
    ROS_INFO("Step 3) pick up the block");
    // 3) PICK UP THE BLOCK
    
    
    ROS_INFO("Step 4) back up (so we don't knock over the stool)");
    // 4) BACK UP
    {
		ros::ServiceClient backwards_client = n.serviceClient<hw_msgs::path>("move_backwards_linearly_service");
		while(!backwards_client.exists())
		{
			ROS_INFO("waiting for service...");
			ros::Duration(1.0).sleep();
		}
		ROS_INFO("connected client to service");
		hw_msgs::path backup_path;
		pose_stamped.pose.position.x = 4.0;
		pose_stamped.pose.position.y = 1.5;
		backup_path.request.path.poses.push_back(pose_stamped);
		backwards_client.call(backup_path);
		
		// WAIT FOR THE ROBOT TO GET THERE
		ros::Duration(10.0).sleep();
	}
	
	ROS_INFO("Step 5) return to the starting pen");
	// 5) RETURN TO STARTING PEN
	hw_msgs::path return_path;
	pose_stamped.pose.position.x = 0.0;
	pose_stamped.pose.position.y = 0.0;
	return_path.request.path.poses.push_back(pose_stamped);
	forward_client.call(return_path);
	
	// WAIT FOR THE ROBOT TO GET THERE
	ros::Duration(25.0).sleep();

    return 0;
}
