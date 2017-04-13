// SYSTEM INCLUDES
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>
#include <eecs376_ps3/ps3Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>


// C++ PROJECT INCLUDES
#include "eecs376_ps3/PathExecutor.h"
#include "eecs376_ps3/movement_commands.h"


const double linear_vel = 1.0; 	// meters / sec
const double yaw_rate = 0.5;	// radians / sec
const double sample_dt = 0.01;	// sample every 10Hz
ros::Publisher global_publisher;


bool path_callback(eecs376_ps3::ps3PathRequest& request, eecs376_ps3::ps3PathResponse& response)
{
	ROS_INFO("callback activated");
    PathExecutor executor;

   	unsigned int waypoints_size = request.nav_path.poses.size();
    ROS_INFO("received path request with %u poses", waypoints_size);


    std::vector<std::pair<double, double> > cmds = executor.generate_commands(request.nav_path.poses);
    ros::Rate loop_timer(1.0 / sample_dt);
    geometry_msgs::Twist twist_cmd;

	double signed_speed = 0.0;
    double total_time = 0.0;

     // initally do nothing for 10 seconds
    zero_twist_command(twist_cmd);
    move_forward_x(0.0, sample_dt, 0.1, loop_timer, global_publisher, twist_cmd);

    ROS_INFO("printing path: {");
    for(unsigned int i = 0; i < cmds.size() && ros::ok(); ++i)
    {
    	ROS_INFO("\tcmd: {angle: %f, distance: %f}", cmds[i].first, cmds[i].second);
    	
    	// make sure to zero out the command first!
    	zero_twist_command(twist_cmd);

    	// take advantage of the fact that a cmd is either a rotation or a translation
    	// (rotation = 0 if a translation, translation = 0 if a rotation)
    	if(cmds[i].first != 0.0) // spin
    	{
    		signed_speed = (cmds[i].first < 0.0 ? -1.0 : 1.0) * yaw_rate;
    		total_time = fabs(cmds[i].first) / yaw_rate;
    		rotate_about_z(signed_speed, sample_dt, total_time, loop_timer, global_publisher, twist_cmd);
    	}
    	else // move forward
    	{
    		signed_speed = (cmds[i].second < 0.0 ? -1.0 : 1.0) * linear_vel;
    		total_time = fabs(cmds[i].second) / linear_vel;
    		move_forward_x(signed_speed, sample_dt, total_time, loop_timer, global_publisher, twist_cmd);
    	}

    	// publish nothing for 0.1 seconds
    	zero_twist_command(twist_cmd);
    	move_forward_x(0.0, sample_dt, 0.1, loop_timer, global_publisher, twist_cmd);


    }
    ROS_INFO("}");

  	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_service");
	ros::NodeHandle handle;
	global_publisher = handle.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);

	ros::ServiceServer service = handle.advertiseService("path_service", path_callback);
	ROS_INFO("Ready to accept paths");
	ros::spin();

	return 0;
}
