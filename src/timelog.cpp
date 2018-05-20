#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ctime>

#include <std_msgs/Int32.h>
#include <gps/Gps.h>

using namespace std;


int bearing;
double latitude;
double longitude;

void bearing_cb(const std_msgs::Int32::ConstPtr& msg)
{ 
  bearing = msg->data;
}

void GPS_cb(const gps::Gps::ConstPtr& msg)  
{   
  latitude = msg->latitude;
  longitude = msg->longitude;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "timelog");
  ros::NodeHandle n;
  ofstream file;
  file.open ("/home/nova/catkin_ws/src/rover/timestamp.txt");

  ros::Subscriber bearing_sub = n.subscribe("/bearing", 1, bearing_cb);
  ros::Subscriber gps_sub = n.subscribe("/gps/gps_data", 1, GPS_cb);
 

  while (ros::ok())
  {
    time_t t = time((time_t*) 0);
    file << ctime(&t) << latitude << ", " << longitude << "\n" << bearing << "\n\n";

    ros::spinOnce();
    ros::Duration(10).sleep(); //Sleep for 10 seconds
  }

  file.close();
  return 0;
}
