// SYSTEM INCLUDES
#include <math.h>


// ROS INCLUDES


// C++ PROJECT INCLUDES
#include "lab_3/PathExecutor.h"


PathExecutor::PathExecutor()
{
}

PathExecutor::~PathExecutor()
{
}

std::vector<std::pair<double, double> > PathExecutor::generate_commands(const std::vector<geometry_msgs::PoseStamped>& waypoints)
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
		angle_to_rotate = this->compute_yaw(current_pose, waypoints[i].pose.position);
		if(angle_to_rotate != 0.0)
		{
			cmds[index_in_cmds++] = std::make_pair(angle_to_rotate, 0.0);
		}

		// move to new coordinates
		distance_to_travel = this->compute_distance(current_pose.position, waypoints[i].pose.position);
		if(distance_to_travel != 0.0)
		{
			cmds[index_in_cmds++] = std::make_pair(0.0, distance_to_travel);
		}

		// THIS IS SLIGHT FEEDBACK!! We could assume that the robot turned perfectly towards its coordinate goal, BUT,
		// I thought it more prudent to actually store what we computed the angle difference to be, and then to take that into account
		// when evaluating spinning for the pose of that coordinate.
		current_pose.orientation = this->convert_angle_to_quaternion(this->convert_quaternion_to_angle(current_pose.orientation) + angle_to_rotate);

		// rotate to desired pose at new coordinates
		angle_to_rotate = this->compute_yaw(current_pose, waypoints[i].pose.orientation);
		if(angle_to_rotate != 0.0)
		{
			cmds[index_in_cmds++] = std::make_pair(angle_to_rotate, 0.0);
		}

		// now we fully assume we are at the desired waypoint
		current_pose = waypoints[i].pose;
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

