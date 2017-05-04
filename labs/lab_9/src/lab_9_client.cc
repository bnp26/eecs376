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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_manipulator/ManipTaskAction.h>
#include <object_manipulation_properties/object_ID_codes.h>
//#include <object_manipulation_properties/object_manipulation_properties.h>
#include <pcl_object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

const double convert_planar_quaternion_to_phi(const geometry_msgs::Quaternion& q)
{
    return 2.0 * atan2(q.z, q.w);
}

bool g_goal_done = true;
int g_callback_status = object_manipulator::ManipTaskResult::PENDING;
int g_object_grabber_return_code=0;
int g_object_finder_return_code=0;
int g_fdbk_count = 0;

geometry_msgs::PoseStamped g_des_flange_pose_stamped_wrt_torso;
geometry_msgs::PoseStamped g_object_pose;
object_manipulator::ManipTaskResult g_result;

void doneCb(const actionlib::SimpleClientGoalState& state,
        const object_manipulator::ManipTaskResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    g_goal_done = true;
    g_result = *result;
    g_callback_status = result->manip_return_code;

    switch (g_callback_status) {
        case object_manipulator::ManipTaskResult::MANIP_SUCCESS:
            ROS_INFO("returned MANIP_SUCCESS");
            
            break;
            
        case object_manipulator::ManipTaskResult::FAILED_PERCEPTION:
            ROS_WARN("returned FAILED_PERCEPTION");
            g_object_finder_return_code = result->object_finder_return_code;
            break;
        case object_manipulator::ManipTaskResult::FAILED_PICKUP:
            ROS_WARN("returned FAILED_PICKUP");
            g_object_grabber_return_code= result->object_grabber_return_code;
            g_object_pose = result->object_pose;
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;
            break;
        case object_manipulator::ManipTaskResult::FAILED_DROPOFF:
            ROS_WARN("returned FAILED_DROPOFF");
            //g_des_flange_pose_stamped_wrt_torso = result->des_flange_pose_stamped_wrt_torso;          
            break;
    }
}

//optional feedback; output has been suppressed (commented out) below
void feedbackCb(const object_manipulator::ManipTaskFeedbackConstPtr& fdbk_msg) {
    g_fdbk_count++;
    if (g_fdbk_count > 1000) { //slow down the feedback publications
        g_fdbk_count = 0;
        //suppress this feedback output
        //ROS_INFO("feedback status = %d", fdbk_msg->feedback_status);
    }
    //g_fdbk = fdbk_msg->feedback_status; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but possibly useful for diagnostics
void activeCb() {
    ROS_INFO("Goal just went active");
}

int acquire_block()
{
	object_manipulator::ManipTaskGoal goal;
	actionlib::SimpleActionClient<object_manipulator::ManipTaskAction> action_client("manip_task_action_service", true);
		
    ROS_INFO("Step 2) find the stool and the block");
    // 2) FIND THE STOOL AND BLOCK
    
    ROS_INFO("Step 3) pick up the block");
    // 3) PICK UP THE BLOCK
        
    // attempt to connect to server
    ROS_INFO("\twaiting for server:");
    bool server_exists = false;
    while((!server_exists) && ros::ok())
    {
		server_exists = action_client.waitForServer(ros::Duration(0.5));
		ros::spinOnce();
		ros::Duration(0.5).sleep();
		ROS_INFO("\tretrying...");
	}
	ROS_INFO("\tconnected to action server");
    
    ROS_INFO("sending a goal: move to pre-pose");
	g_goal_done = false;
	goal.action_code = object_manipulator::ManipTaskGoal::MOVE_TO_PRE_POSE;
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	while (!g_goal_done && ros::ok()) {
		ros::Duration(0.1).sleep();
	}
	if (g_callback_status!= object_manipulator::ManipTaskResult::MANIP_SUCCESS)
	{
		ROS_ERROR("failed to move quitting");
		return 0;
	}
	//send vision request to find table top:
	ROS_INFO("sending a goal: seeking table top");
	g_goal_done = false;
	goal.action_code = object_manipulator::ManipTaskGoal::FIND_TABLE_SURFACE;

	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	while (!g_goal_done && ros::ok()) {
		ros::Duration(0.1).sleep();
	}
    
	//send vision goal to find block:
	ROS_INFO("sending a goal: find block");
	g_goal_done = false;
	goal.action_code = object_manipulator::ManipTaskGoal::GET_PICKUP_POSE;
	goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
	goal.perception_source = object_manipulator::ManipTaskGoal::PCL_VISION;
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	while (!g_goal_done && ros::ok()) {
		ros::Duration(0.1).sleep();
	}    
	if (g_callback_status!= object_manipulator::ManipTaskResult::MANIP_SUCCESS)
	{
		ROS_ERROR("failed to find block quitting");
		return 0;
	}
	g_object_pose = g_result.object_pose;
	ROS_INFO_STREAM("object pose w/rt frame-id "<<g_object_pose.header.frame_id<<endl);
	ROS_INFO_STREAM("object origin: (x,y,z) = ("<<g_object_pose.pose.position.x<<", "<<g_object_pose.pose.position.y<<", "
			<<g_object_pose.pose.position.z<<")"<<endl);

    g_object_pose.pose.orientation = convertPlanarPhi2Quaternion(convert_planar_quaternion_to_phi(g_object_pose.pose.orientation) + 1.57);

	ROS_INFO_STREAM("orientation: (qx,qy,qz,qw) = ("<<g_object_pose.pose.orientation.x<<","
			<<g_object_pose.pose.orientation.y<<","
			<<g_object_pose.pose.orientation.z<<","
			<<g_object_pose.pose.orientation.w<<")"<<endl); 
    
    
	//send command to acquire block:
	ROS_INFO("sending a goal: grab block");
	g_goal_done = false;
	goal.action_code = object_manipulator::ManipTaskGoal::GRAB_OBJECT;
	goal.pickup_frame = g_result.object_pose;
	goal.object_code= ObjectIdCodes::TOY_BLOCK_ID;
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	while (!g_goal_done && ros::ok()) {
		ros::Duration(0.1).sleep();
	}    
	if (g_callback_status!= object_manipulator::ManipTaskResult::MANIP_SUCCESS)
	{
		ROS_ERROR("failed to grab block; quitting");
		return 0;
	}
    
	ROS_INFO("sending a goal: move to pre-pose");
	g_goal_done = false;
	goal.action_code = object_manipulator::ManipTaskGoal::MOVE_TO_PRE_POSE;
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	while (!g_goal_done && ros::ok()) {
		ros::Duration(0.1).sleep();
	}
	if (g_callback_status!= object_manipulator::ManipTaskResult::MANIP_SUCCESS)
	{
		ROS_ERROR("failed to move to pre-pose; quitting");
		return 0;
	}    
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
    
    const double stool_x = 3.77;
    const double stool_y = 1.95;
    
    ROS_INFO("Step 1) navigate to the stool");
    // 1) NAVIGATE TO THE STOOL
    quat = convertPlanarPhi2Quaternion(0);
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation = quat;
    
    pose_stamped.pose.position.x = stool_x;
    pose_stamped.pose.position.y = 0;
    path.request.path.poses.push_back(pose_stamped);

    pose_stamped.pose.position.x = stool_x;
    pose_stamped.pose.position.y = stool_y; 
    path.request.path.poses.push_back(pose_stamped);
    forward_client.call(path);
    
    // WAIT FOR THE ROBOT TO GET THERE
    ros::Duration(45.0).sleep();
    
	acquire_block();
    
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
		pose_stamped.pose.position.x = stool_x;
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
