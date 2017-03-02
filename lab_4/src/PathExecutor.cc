// SYSTEM INCLUDES
#include <math.h>


// ROS INCLUDES
#include <geometry_msgs/Twist.h>


// C++ PROJECT INCLUDES
#include "lab_4/PathExecutor.h"


PathExecutor::PathExecutor(const double linear_vel, const double yaw_rate,
						   const double sample_dt, std::string pub_topic,
						   ros::NodeHandle* p_handle,
						   actionlib::SimpleActionServer<lab_4::PathAction>* p_action_server) :
	_linear_vel(linear_vel), _yaw_rate(yaw_rate), _sample_dt(sample_dt),
	_pub_topic(pub_topic), _publisher(p_handle->advertise<geometry_msgs::Twist>(pub_topic.c_str(), 1)),
	_timer(1.0 / this->_sample_dt), _p_action_server(p_action_server),
	_p_alarm_subscriber(p_handle->subscribe("/lidar_alarm", 1, &PathExecutor::alarm_callback, this)),
	_alarm_trigger(false)
{
}

PathExecutor::~PathExecutor()
{
}

std::vector<std::pair<double, double> > PathExecutor::generate_commands(const std::vector<geometry_msgs::Pose>& waypoints)
{
	std::vector<std::pair<double, double> > cmds;
	cmds.resize(3 * waypoints.size()); // need at max 3 commands for every waypoint

	geometry_msgs::Pose current_pose = this->zero_pose();

	double angle_to_rotate = 0.0;
	double distance_to_travel = 0.0;

	unsigned int index_in_cmds = 0;
	for(unsigned int i = 0; i < waypoints.size(); ++i)
	{
		// rotate to point towards new coordinates
		angle_to_rotate = this->compute_yaw(current_pose, waypoints[i].position);
		if(angle_to_rotate != 0.0)
		{
			cmds[index_in_cmds++] = std::make_pair(angle_to_rotate, 0.0);
		}

		// move to new coordinates
		distance_to_travel = this->compute_distance(current_pose.position, waypoints[i].position);
		if(distance_to_travel != 0.0)
		{
			cmds[index_in_cmds++] = std::make_pair(0.0, distance_to_travel);
		}

		// THIS IS SLIGHT FEEDBACK!! We could assume that the robot turned perfectly towards its coordinate goal, BUT,
		// I thought it more prudent to actually store what we computed the angle difference to be, and then to take that into account
		// when evaluating spinning for the pose of that coordinate.
		current_pose.orientation = this->convert_angle_to_quaternion(this->convert_quaternion_to_angle(current_pose.orientation) + angle_to_rotate);

		// rotate to desired pose at new coordinates
		angle_to_rotate = this->compute_yaw(current_pose, waypoints[i].orientation);
		if(angle_to_rotate != 0.0)
		{
			cmds[index_in_cmds++] = std::make_pair(angle_to_rotate, 0.0);
		}

		// now we fully assume we are at the desired waypoint
		current_pose = waypoints[i];
	}

	// resize vector....if we did not use a command (rotate 0 radians is useless, move forward 0 meters is useless....then our
	// vector has a bunch of useless commands at the end, so prune them away)
	if(index_in_cmds < cmds.size())
	{
		cmds.resize(index_in_cmds);
	}

	return cmds;
}

geometry_msgs::Quaternion PathExecutor::zero_quaternion()
{
	geometry_msgs::Quaternion q;
	q.x = 0.0;
	q.y = 0.0;
	q.z = 0.0;
	q.w = 1.0;
	return q;
}

geometry_msgs::Pose PathExecutor::zero_pose()
{
	geometry_msgs::Pose p;
	p.position.x = 0.0;
	p.position.y = 0.0;
	p.position.z = 0.0;

	p.orientation = this->zero_quaternion();

	return p;
}

const double PathExecutor::convert_quaternion_to_angle(const geometry_msgs::Quaternion& q)
{
	return 2.0 * atan2(q.z, q.w);
}

const geometry_msgs::Quaternion PathExecutor::convert_angle_to_quaternion(const double phi)
{
	geometry_msgs::Quaternion q = this->zero_quaternion();
	q.z = sin(phi / 2.0);
	q.w = cos(phi / 2.0);
	return q;
}

double PathExecutor::min_spin(const double angle)
{
	double min_angle = angle;
	if(angle > PI)
	{
		min_angle -= 2.0 * PI;
	}
	if(angle < -PI)
	{
		min_angle += 2.0 * PI;
	}
	return min_angle;
}

double PathExecutor::compute_yaw(const geometry_msgs::Pose& current_pose, const geometry_msgs::Quaternion& next_orientation)
{
	double next_yaw = this->convert_quaternion_to_angle(next_orientation);
	double current_yaw = this->convert_quaternion_to_angle(current_pose.orientation);
	return this->min_spin(next_yaw - current_yaw);
}

double PathExecutor::compute_yaw(const geometry_msgs::Pose& current_pose, const geometry_msgs::Point& to_point_at)
{
	double current_yaw = this->convert_quaternion_to_angle(current_pose.orientation);
	
	// oh god doing vector math but by hand
	double current_vector[] = {cos(current_yaw), sin(current_yaw), 0.0};

	double vector_between_pos[] = {to_point_at.x - current_pose.position.x,
								   to_point_at.y - current_pose.position.y,
								   to_point_at.z - current_pose.position.z};

	double dot_product = current_vector[0] * vector_between_pos[0] +
						 current_vector[1] * vector_between_pos[1] +
						 current_vector[2] * vector_between_pos[2];
	double magnitude_current_vector = sqrt(current_vector[0] * current_vector[0] +
									       current_vector[1] * current_vector[1] +
									       current_vector[2] * current_vector[2]);
	double magnitude_point_vector = sqrt(vector_between_pos[0] * vector_between_pos[0] +
									     vector_between_pos[1] * vector_between_pos[1] +
									     vector_between_pos[2] * vector_between_pos[2]);

	double angle_diff = acos(dot_product / (magnitude_point_vector * magnitude_current_vector));
	return this->min_spin(angle_diff);
}

double PathExecutor::compute_distance(const geometry_msgs::Point& start, const geometry_msgs::Point& end)
{
	double x_dist = end.x - start.x;
	double y_dist = end.y - start.y;
	double z_dist = end.z - start.z;

	double dist = sqrt(x_dist * x_dist + y_dist * y_dist + z_dist * z_dist);

	// assumed robot only moves forwards...otherwise force negative if we want to move backwards.
	// dist *= (x_dist < 0.0 || y_dist < 0.0 || z_dist < 0.0) ? -1.0 : 1.0;
	return dist;
}


void PathExecutor::Execute_Command(const std::pair<double, double>& cmd)
{
	geometry_msgs::Twist twist_cmd;

	this->zero_twist_command(twist_cmd);

	double signed_speed = 0.0;
	double total_time = 0.0;

	ROS_INFO("executing cmd: (%f, %f)", cmd.first, cmd.second);
	if(cmd.first != 0.0) // spin
	{
		signed_speed = (cmd.first < 0.0 ? -1.0 : 1.0) * this->_yaw_rate;
		total_time = fabs(cmd.first) / this->_yaw_rate;
		this->rotate_about_z(signed_speed, this->_sample_dt, total_time,
					         twist_cmd);
	}
	else
	{
		signed_speed = (cmd.second < 0.0 ? -1.0 : 1.0) * this->_linear_vel;
		total_time = fabs(cmd.second) / this->_linear_vel;
		this->move_forward_x(signed_speed, this->_sample_dt, total_time,
				   			 twist_cmd);
	}

	this->zero_twist_command(twist_cmd);
	this->move_forward_x(0.0, this->_sample_dt, 0.1, twist_cmd);
}

void PathExecutor::Send_Zero_Command()
{
	geometry_msgs::Twist twist_cmd;
	this->zero_twist_command(twist_cmd);
	this->_publisher.publish(twist_cmd);
}

void PathExecutor::alarm_callback(const std_msgs::Bool& alarm_msg)
{
	this->_alarm_trigger = alarm_msg.data;
	if(this->_alarm_trigger)
	{
		ROS_INFO("LIDAR ALARM!!!");
	}
}

geometry_msgs::Twist& PathExecutor::zero_twist_command(geometry_msgs::Twist& cmd)
{
	// set linear components to 0
	cmd.linear.x = 0.0;
	cmd.linear.y = 0.0;
	cmd.linear.z = 0.0;

	// set angular components to 0
	cmd.angular.x = 0.0;
	cmd.angular.y = 0.0;
	cmd.angular.z = 0.0;

	// safe to return reference
	return cmd;
}

double PathExecutor::move(double dt, double total_time, geometry_msgs::Twist& cmd, bool override)
{
	double elapsed_time = 0.0;
	geometry_msgs::Twist zero_cmd;
	this->zero_twist_command(zero_cmd);

	while(elapsed_time < total_time && ros::ok())
	{
		ros::spinOnce();
		while(!override && ros::ok() && this->_alarm_trigger)
		{
			this->_publisher.publish(zero_cmd);
			ros::spinOnce();	
		}
		this->_publisher.publish(cmd);
		elapsed_time += dt;
		this->_timer.sleep();
	}

	return elapsed_time;
}

double PathExecutor::rotate_about_z(double speed, double dt, double total_time, geometry_msgs::Twist& cmd)
{
	this->zero_twist_command(cmd);
	cmd.angular.z = speed;

	return this->move(dt, total_time, cmd, true);
}

double PathExecutor::move_forward_x(double speed, double dt, double total_time, geometry_msgs::Twist& cmd)
{
	this->zero_twist_command(cmd);
	cmd.linear.x = speed;

	return this->move(dt, total_time, cmd, false);
}
