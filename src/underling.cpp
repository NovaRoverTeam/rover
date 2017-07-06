#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>
//#include "rover/RawControl.h"
//#include "rover/Command.h"

class PubScrub
{
public:
  PubScrub()
  {
    pub_ = n_.advertise<rover::Command>("command_data", 1000);

    scrub_ = n_.subscribe("control_data", 1000, &PubScrub::control_dataCallback, this);
  }

  void control_dataCallback(const rover::RawControl::ConstPtr& msg)
  {
    //ROS_INFO("Axis %d has value %.5f", msg->id, msg->length);

    rover::Command cmd_msg;
    cmd_msg.system = 0;
    cmd_msg.target = 0;

    bool driving = false;

    
    ROS_INFO_STREAM(msg->axis_x); 
    ROS_INFO_STREAM(fabs(msg->axis_y)); 

    if (fabs(msg->axis_y) > stick_threshold) 
    {
      driving = true;
      //ROS_INFO("DRIVE");
      cmd_msg.command = 1; // Drive
      cmd_msg.value1 = msg->axis_y;
      cmd_msg.value2 = msg->axis_y;
    }
    
    if ((!driving) && (fabs(msg->axis_x) > stick_threshold))
    {
      //ROS_INFO("STEER");
      cmd_msg.command = 2; // Steer
      cmd_msg.value1 = -(msg->axis_x);
      cmd_msg.value2 = msg->axis_x;
    }
  
    pub_.publish(cmd_msg);      
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber scrub_;

}; // END OF CLASS - PubScrub

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underling");

  PubScrub PS; // Create PubScrub object

  ros::spin();

  return 0;
}

