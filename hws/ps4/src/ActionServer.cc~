// SYSTEM INCLUDES
#include <utility>
#include <vector>

// C++ PROJECT INCLUDES
#include "eecs376_ps4/ActionServer.h"


ActionServer::ActionServer(const std::string& action_server_topic,
						   const double linear_vel, const double yaw_rate,
						   const double sample_dt, const std::string pub_topic) : _handle(),
	_action_server(this->_handle, action_server_topic.c_str(),
				   boost::bind(&ActionServer::Execute_Callback, this, _1), false),
	_path_goals(), _path_response(), _path_feedback(),
	_path_executor(linear_vel, yaw_rate, sample_dt, pub_topic, &this->_handle, &this->_action_server)
{
	this->_action_server.start();
}

ActionServer::~ActionServer()
{
}

void ActionServer::Execute_Callback(const actionlib::SimpleActionServer<eecs376_ps4::PathAction>::GoalConstPtr& goal)
{
	unsigned int goal_poses_size = goal->goal_poses.size();
	ROS_INFO("receiving path with %u elements", goal_poses_size);

	std::vector<std::pair<double, double> > path_commands =
		this->_path_executor.generate_commands(goal->goal_poses);

	geometry_msgs::Pose current_pose;
	for(unsigned int i = 0; i < path_commands.size(); ++i)
	{
		if(this->_action_server.isPreemptRequested())
		{
			ROS_INFO("setting continue_moving to false");
			this->Cancel_Goal();
			return;
		}

		this->_path_executor.Execute_Command(path_commands[i]);
		this->_path_feedback.current_percentage = i;

		this->_action_server.publishFeedback(this->_path_feedback);
	}

	this->_path_response.result_pose_stamp = goal->goal_poses.size();
	this->_action_server.setSucceeded(this->_path_response);

}

void ActionServer::Cancel_Goal()
{
	this->_path_response.result_pose_stamp = -1;
	this->_path_executor.Send_Zero_Command();
	this->_action_server.setAborted(this->_path_response);
}
