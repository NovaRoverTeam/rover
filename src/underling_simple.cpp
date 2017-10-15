#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>
#include <stdlib.h>

#include <iostream>
using namespace std;

#include <wiringPi.h>
#include <pca9685.h>
//#include <miniPID.h>

#define K_P 5
#define K_I 0
#define K_D 0

#define PIN_BASE 160
#define MAX_PWM 4096
#define PWM_HERTZ 1000
#define MAX_RPM 118 // max RPM of servos from data sheet

#define LOOP_HERTZ 10 // Main control loop rate

// Pin definitions for direction-changing GPIOs
#define B_L_DIR_PIN 4 // 23
#define F_R_DIR_PIN 5 // 24
#define F_L_DIR_PIN 23 // 13
#define B_R_DIR_PIN 24 // 19

#define F_STR_PIN 27 // 16
#define B_STR_PIN 28 // 20

#include <rover/DriveCmd.h>
#include <std_msgs/Empty.h>

float drive_pcnt = 0; // Desired wheel speed percentage
float steer_pcnt = 0; // Desired steering speed percentage

bool alive = false; // True if we have contact with mainframe
int hbeat_cnt = 0; // Counter of how many loops have passed since heartbeat
const float iteration_time = 1/LOOP_HERTZ;

// Clamp value within range - convenience function
int clamp(int value, int max, int min)
{
  if (value > max) return max;
  else if (value < min) return min;
  else return value;
}

// Roll value over range - convenience function
int rollover(int value, int max, int min)
{
  if (value > max) return min;
  else if (value < min) return max;
  else return value;
}


// Callback for subscription to drive command topic "cmd_data"
void cmd_data_cb(const rover::DriveCmd::ConstPtr& msg)
{    
    if (alive)
    {
      drive_pcnt = msg->acc;        // Store desired angular acceleration as %
      steer_pcnt = 100.0*(msg->steer)/45; // Store desired steering angle as %
      for(int i=0;i<4;i++) 
      {
          req_RPM[i] = abs((drive_pcnt/100)*MAX_RPM);
      }

      ROS_INFO_STREAM("Drive command received.");
    }
}

// Callback for heartbeat subscription
void hbeat_cb(const std_msgs::Empty::ConstPtr& msg)
{
  alive = true;
  hbeat_cnt = 0;
}

int main(int argc, char **argv)
{
  // ********************* ROS ************************* //

  ros::init(argc, argv, "underling");
  ros::NodeHandle n;
  ros::Rate loop_rate(LOOP_HERTZ);

  ros::Subscriber drivecmd_sub = n.subscribe("/mainframe/cmd_data", 5, cmd_data_cb);	
  ros::Subscriber hbeat_sub = n.subscribe("hbeat", 1, hbeat_cb);	

  // If no heartbeat for 2 seconds, rover dies
  const int hbeat_timeout = 2*LOOP_HERTZ;

  // ****************** VARIABLES ********************** //

  // Format is [f_l, f_r, b_l, b_r]
  const int dir_pins[4] = {F_L_DIR_PIN, F_R_DIR_PIN, B_L_DIR_PIN, B_R_DIR_PIN};  
  const bool correction[4] = {0, 1, 0, 1}; // Correct motor directions

  int req_RPM[4] = {0,0,0,0};
  int actual_RPM[4] = {0,0,0,0};
  int error[4] = {0,0,0,0};
  int error_prior[4] = {0,0,0,0};
  float integral[4] = {0,0,0,0};
  float derivative[4] = {0,0,0,0};
  int output[4] = {0,0,0,0};
  int drive_pwm[4] = {0,0,0,0};

  //int drive_pwm = 0;  // Wheel PWM
  int steer_pwm = 0;  // Wheel PWM

  bool drive_dir = 1; // Wheel direction
  bool steer_dir = 1; // Steering direction

  float limit_drive = 0.3;
  float limit_steer = 0.5;

  if (argc == 3)
  {
    limit_drive = atof(argv[1]);
    limit_steer = atof(argv[2]);
  }

  // ******************** SETUP ************************ //

  int fd = pca9685Setup(PIN_BASE, 0x40, PWM_HERTZ);
  if (fd < 0)
  {
    printf("Error in setup\n");
    return fd;
  }

  pca9685PWMReset(fd); // Reset all outputs

  wiringPiSetup();

  // Configure GPIOs and set initial directions
  for (int i = 0; i < 4; i++) 
  {
    pinMode (dir_pins[i], OUTPUT);
    digitalWrite (dir_pins[i], drive_dir);
  }

  pinMode (F_STR_PIN, OUTPUT);
  pinMode (B_STR_PIN, OUTPUT);
  digitalWrite (B_STR_PIN, steer_dir);
  digitalWrite (F_STR_PIN, !steer_dir);

  // ****************** MAIN LOOP ********************** //

  while (ros::ok())
  {
    // If no heartbeat, kill rover
    if (hbeat_cnt > hbeat_timeout) 
    {
      alive = false; 
      ROS_INFO_STREAM("No heartbeat - killing rover.");   
    }

    // Set new motor directions
    drive_dir = !(drive_pcnt < 0);
    steer_dir = !(steer_pcnt < 0);

    if (alive)
    {
      for(int k=0;k<4;k++) {
          error[k] = req_RPM[k] - actual_RPM[k];
	  integral[k] = integral[k] + (error[k] * iteration_time);
          derivative[k] = (error[k] - error_prior[k]) / iteration_time;
          output[k] = (K_P*error[k]) + (K_I*integral[k]) + (K_D*derivative[k]);
          error_prior[k] = error[k];

          drive_pwm[k] = limit_drive*output[k];
          drive_pwm[k] = clamp(drive_pwm[k], MAX_RPM, 0);
      }

      // Map drive percentage to PWM as a quadratic, with limit
      //drive_pwm = limit_drive*MAX_PWM*pow(drive_pcnt/100, 2);
      
      // Make sure the PWM val hasn't gone outside range somehow
      //drive_pwm = clamp(drive_pwm, MAX_PWM, 0);

      // Map steering percentage to PWM as a quadratic, with limit
      steer_pwm = limit_steer*MAX_PWM*pow(steer_pcnt/100, 2);

      // Make sure the PWM val hasn't gone outside range somehow
      steer_pwm = clamp(steer_pwm, MAX_PWM, -MAX_PWM);
    }
    else 
    {
      // Stop rover if dead
      for(int j = 0; j < 4; j++) {
          drive_pwm[j] = 0;
      }
      steer_pwm = 0;
    }

    for (int i = 0; i < 4; i++)
    {
      // Correct wheel directions
      digitalWrite (dir_pins[i], correction[i] != drive_dir);

      // Change wheel speed outputs
      pwmWrite(PIN_BASE + i, drive_pwm[i]); // pins of PWM board, (0, 1, 2, 3)
    }

    digitalWrite (B_STR_PIN, steer_dir);
    digitalWrite (F_STR_PIN, !steer_dir);

    pwmWrite(PIN_BASE + 4, steer_pwm); // pins of PWM board, (4, 5 for steering)
    pwmWrite(PIN_BASE + 5, steer_pwm);

    hbeat_cnt++; // Increment heartbeat tracking timer

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
