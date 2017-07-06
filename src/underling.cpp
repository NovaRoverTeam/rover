#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 50

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underling");
  ros::Rate loop_rate(50); // 50 Hz

  bool increasing = true;
  int value = 1000;

  pca9685Setup(PIN_BASE, 0x00, HERTZ);

  while (ros::ok())
  {
    pwmWrite(PIN_BASE + 4, value);
  
    if (increasing) 
      value += 50;
    else
      value -= 50;
    
    if (value > 3000)
      increasing = false;
    else if (value < 1000)
      increasing = true;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

