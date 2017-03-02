#pragma once
#ifndef LAB_4_ACTION_SERVER
#define LAB_4_ACTION_SERVER


// SYSTEM INCLUDES
#include <string>

// ROS INCLUDES
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// C++ PROJECT INCLUDES
#include "lab_4/PathAction.h"
#include "lab_4/PathExecutor.h"


class ActionServer
{
public:

	ActionServer(const std::string& action_server_topic,
			     const double linear_vel, const double yaw_rate,
			     const double sample_dt, const std::string pub_topic);

	virtual ~ActionServer();

	void Execute_Callback(const actionlib::SimpleActionServer<lab_4::PathAction>::GoalConstPtr& goal);

protected:

	void Cancel_Goal();

private:

	ros::NodeHandle											_handle;
	actionlib::SimpleActionServer<lab_4::PathAction>		_action_server;
	lab_4::PathGoal											_path_goals;
	lab_4::PathResult										_path_response;
	lab_4::PathFeedback										_path_feedback;
	PathExecutor											_path_executor;

};

#endif // end of LAB_4_ACTION_SERVER
