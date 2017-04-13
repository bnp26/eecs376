// SYSTEM INCLUDES


// ROS INCLUDES
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <lab_4/PathAction.h>

// C++ PROJECT INCLUDES
//#include "lab_4/PathAction.h"
#include "lab_4/waypoint_generator.h"


#define PI 3.141592653589793238462643383279502884

lab_4::PathGoal global_plan;
int progress_made = 0;

void done_callback(const actionlib::SimpleClientGoalState& state,
				   const lab_4::PathResultConstPtr& result)
{
	ROS_INFO("done_callback: server responded with state [%s]", state.toString().c_str());
	ROS_INFO("got result output = %d", result->result_pose_stamp);
}


void feedback_callback(const lab_4::PathFeedbackConstPtr& feedback)
{
	ROS_INFO("feedback status = %d", feedback->current_percentage);
	progress_made = feedback->current_percentage;
}

void active_callback()
{
	ROS_INFO("goal just went active");
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "action_client_node");
	ros::NodeHandle handle;
	ros::Rate timer(1.0);

	actionlib::SimpleActionClient<lab_4::PathAction> action_client("path_action_server", true);

	ROS_INFO("attempting to connect to server");

	while(!action_client.waitForServer(ros::Duration(1.0)))
	{
		ROS_WARN("could not connect to server...retrying in 1 second");
	}
	ROS_INFO("connected to action server");

	// ROS_INFO("cancelling goal");
	// action_client.cancelGoal();

	global_plan.goal_poses.push_back(generate_waypoint(6.0, 0.0, 0));
	global_plan.goal_poses.push_back(generate_waypoint(6.0, 1.5, PI / 2));

	action_client.sendGoal(global_plan, &done_callback,
						   &active_callback, &feedback_callback);

	ros::spin();

	return 0;
}
