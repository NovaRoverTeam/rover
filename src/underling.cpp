#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 50

#define MAX_PULSE 600
#define MIN_PULSE 150

#include "gamepad/gamepad.h"

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

  pca9685PWMReset(fd); // Reset all output

  GamepadInit(); // Initialise the Xbox gamepad

  while (ros::ok())
  {
    GamepadUpdate();

    int l_stick_x = 0;
    int l_stick_y = 0;

    GamepadStickXY(GAMEPAD_0, STICK_LEFT, &l_stick_x, &l_stick_y);

    float stick_x = ((float) l_stick_x)/32767;
    float stick_y = ((float) l_stick_y)/32767;

    int pulse_12 = (int)(MIN_PULSE + stick_x*(MAX_PULSE-MIN_PULSE));
    int pulse_13 = (int)(MIN_PULSE + stick_y*(MAX_PULSE-MIN_PULSE));

    pwmWrite(PIN_BASE + 12, pulse_12);
    pwmWrite(PIN_BASE + 13, pulse_13);

    /*
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
    */

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

