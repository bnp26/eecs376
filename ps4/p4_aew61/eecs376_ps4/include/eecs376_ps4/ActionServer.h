#pragma once
#ifndef EECS376_PS4_ACTION_SERVER
#define EECS376_PS4_ACTION_SERVER


// SYSTEM INCLUDES
#include <string>

// ROS INCLUDES
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// C++ PROJECT INCLUDES
#include "eecs376_ps4/PathAction.h"
#include "eecs376_ps4/PathExecutor.h"


class ActionServer
{
public:

	ActionServer(const std::string& action_server_topic,
			     const double linear_vel, const double yaw_rate,
			     const double sample_dt, const std::string pub_topic);

	virtual ~ActionServer();

	void Execute_Callback(const actionlib::SimpleActionServer<eecs376_ps4::PathAction>::GoalConstPtr& goal);

protected:

	void Cancel_Goal();

private:

	ros::NodeHandle											_handle;
	actionlib::SimpleActionServer<eecs376_ps4::PathAction>	_action_server;
	eecs376_ps4::PathGoal									_path_goals;
	eecs376_ps4::PathResult									_path_response;
	eecs376_ps4::PathFeedback								_path_feedback;
	PathExecutor											_path_executor;

};

#endif // end of EECS376_PS4_ACTION_SERVER
