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
    ros::ServiceClient client = n.serviceClient<hw_msgs::path>("append_path_queue_service");
    ros::ServiceClient flush_client = n.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    hw_msgs::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    
    quat = convertPlanarPhi2Quaternion(0);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation = quat;
    
    pose_stamped.pose.position.x = 9.47035371435 + 5.0;
    pose_stamped.pose.position.y = 0;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x = 9.47035371435 + 5.0;
    pose_stamped.pose.position.y = -30;
    pose_stamped.pose.position.z = 0.0;    
    path_srv.request.path.poses.push_back(pose_stamped);

    
    pose_stamped.pose.position.x = 9.47035371435 + 5.0;
    pose_stamped.pose.position.y = 0;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x = 9.47035371435;
    pose_stamped.pose.position.y = 0.0;
    pose_stamped.pose.orientation = convertPlanarPhi2Quaternion(3.14157);
    path_srv.request.path.poses.push_back(pose_stamped);
   
    client.call(path_srv);

    return 0;
}
