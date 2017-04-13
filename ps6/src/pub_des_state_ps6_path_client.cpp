//pub_des_state_path_client:
// illustrates how to send a request to the append_path_queue_service service

#include <ros/ros.h>
#include <zeta_planners/path.h>
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
    ros::ServiceClient client = n.serviceClient<zeta_planners::path>("append_path_queue_service");
    ros::ServiceClient flush_client = n.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    zeta_planners::path path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose pose;
    pose.position.x = 5.0; // say desired x-coord is 5
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    pose.orientation = quat;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
 
    pose.position.y = 5.0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose.position.x = 0.0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);

    pose.position.y = 0.0;
    pose_stamped.pose = pose;
    path_srv.request.path.poses.push_back(pose_stamped);
    
    //repeat (x,y) with new heading:
    pose_stamped.pose.orientation = convertPlanarPhi2Quaternion(0); 
    path_srv.request.path.poses.push_back(pose_stamped);
    
    client.call(path_srv);

    zeta_planners::path new_path;
    geometry_msgs::PoseStamped new_pose_stamped;
    new_pose_stamped.header.frame_id = "world";
    geometry_msgs::Pose new_pose;
    new_pose.position.x = 15.0; // say desired x-coord is 5
    new_pose.position.y = 0.0;
    new_pose.position.z = 0.0; // let's hope so!
    quat = convertPlanarPhi2Quaternion(0);
    new_pose.orientation = quat;
    new_pose_stamped.pose = new_pose;
    new_path.request.path.poses.push_back(new_pose_stamped);


    ROS_WARN("LETTING PATH EXECUTE FOR 100 SECONDS AND THEN FLUSHING!");
    ros::Duration(100.0).sleep();
    ROS_WARN("FLUSHING PATH");
    std_srvs::Trigger flush_trigger;
    flush_client.call(flush_trigger);

    ROS_WARN("WAITING 10.0 SECONDS BEFORE SENDING NEW PATH");
    ros::Duration(10.0).sleep();

    ROS_WARN("SENDING NEW PATH!");
    client.call(new_path);

    return 0;
}
