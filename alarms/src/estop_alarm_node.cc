#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

bool b_estop;
ros::ServiceClient c;

void estop_callback(const std_msgs::Bool& msg)
{
    if(msg.data)
    {
        ROS_INFO("ESTOP called!");
        std_srvs::Trigger trigger;
        c.call(trigger);
    }
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "zeta_lab_6_estop_node");
   ros::NodeHandle handle;

   ros::ServiceClient client = handle.serviceClient<std_srvs::Trigger>("estop_service");
   while(!client.exists() && ros::ok())
   {
       ROS_INFO("waiting_for_service");
       ros::Duration(1.0).sleep();
   }
   if(client.exists())
   {
       c = client;
       ros::Subscriber sub = handle.subscribe("/ESTOP", 1, estop_callback);
       ros::spin();
   }
   return 0;
}

