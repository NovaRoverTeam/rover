#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ctime>

#include <std_msgs/Int32.h>

using namespace std;

#define LOOP_HERTZ 20

int bearing;

void bearing_cb(const std_msgs::Int32::ConstPtr& msg)
{ 
  bearing = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timelog");
  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_HERTZ);
  ofstream file;
  file.open ("/home/nova/catkin_ws/src/rover/timestamp.txt");

  ros::Subscriber abc_sub = n.subscribe("/bearing", 1, bearing_cb);  

  while (ros::ok())
  {
    time_t t = time((time_t*) 0);
    file << ctime(&t) << bearing << "\n\n";

    ros::spinOnce();
    loop_rate.sleep();
  }

  file.close();
  return 0;
}
