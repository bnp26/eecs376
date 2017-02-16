// SYSTEM INCLUDES
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean message
#include <stdlib.h>
#include <time.h>


// C++ PROJECT INCLUDES
#include "lab_2/movement_commands.h"


bool g_lidar_alarm = false; // global var for lidar alarm

void alarm_callback(const std_msgs::Bool& alarm_msg) 
{ 
  g_lidar_alarm = alarm_msg.data; //make the alarm status global, so main() can use it
  if (g_lidar_alarm)
  {
     ROS_INFO("LIDAR alarm received!"); 
  }
} 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander"); 
    ros::NodeHandle n;
    ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
    ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm", 1, alarm_callback); 
    
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 0.5; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds

    srand(time(NULL)); // seed the RNG
    int yaw_direction = 0; // flag for the direction of rotation

    geometry_msgs::Twist twist_cmd; // this is the message type required to send twist commands to STDR 

    zero_twist_command(twist_cmd);  // zero out the command so we don't tell the robot to do anything

    ros::Rate loop_timer(1/sample_dt); // create a ros object from the ros “Rate” class; set 100Hz rate     
    double timer = 0.0;

    //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i = 0; i < 10; ++i)
    {
      twist_commander.publish(twist_cmd);
      ros::spinOnce();
      loop_timer.sleep();
    }


    while(ros::ok())
    {
    	// keep moving forward unitl an alarm is triggered.
    	// Note that we NEED the "&& ros::ok()" part so the code will fall out of the loop when
    	// roscore is shut down and we can Ctrl-C.
        while(!g_lidar_alarm && ros::ok())
        {
          	move_forward_x(speed, twist_commander, twist_cmd);
          	timer += sample_dt;
          	ros::spinOnce();
			loop_timer.sleep();
        }

        //here if got an alarm; turn CCW until alarm clears
        timer = 0.0; //reset the timer
        yaw_direction = ((rand() % 3 - 1) < 0.0) ? -1 : 1;

        // Note that we NEED the "&& ros::ok()" part so the code will fall out of the loop when
    	// roscore is shut down and we can Ctrl-C.
        while(g_lidar_alarm && ros::ok())
        {
          	rotate_about_z(yaw_direction * yaw_rate, twist_commander, twist_cmd);
          	timer += sample_dt;
          	ros::spinOnce();
          	loop_timer.sleep();
        }
  	}          

    return 0;
}
