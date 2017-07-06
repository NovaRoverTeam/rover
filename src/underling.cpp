#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 50


// Calculate the number of ticks the signal should be high for the required amount of time
int calcTicks(float impulseMs, int hertz)
{
  float cycleMs = 1000.0f / hertz;
  return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underling");
  ros::NodeHandle n;
  ros::Rate loop_rate(50); // 50 Hz

  bool increasing = true;
  int value = 200;

  int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
  if (fd < 0)
  {
    printf("Error in setup\n");
    return fd;
  }

  // Reset all output
  pca9685PWMReset(fd);

  while (ros::ok())
  {
    pwmWrite(PIN_BASE + 4, value);
  
    if (increasing) 
      value += 1;
    else
      value -= 1;
    
    if (value > 400)
      increasing = false;
    else if (value < 200)
      increasing = true;
	
    ROS_INFO_STREAM(value);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

