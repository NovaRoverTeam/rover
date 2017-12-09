#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>

#include <iostream>
using namespace std;

#include <wiringPi.h>
#include "pca9685/src/pca9685.h" // PWM board library
//#include <miniPID.h>

// Default K values
#define K_P 0
#define K_I 0
#define K_D 0

#define PIN_BASE 160
#define PWM_HERTZ 1000

#define MAX_PWM 4096 // Max PWM of pca9685
#define MAX_RPM 118 // Max RPM of motors from data sheet

#define LOOP_HERTZ 10 // Main control loop rate

// Pin definitions for direction-changing GPIOs
#define B_L_DIR_PIN 4 // 23
#define F_R_DIR_PIN 5 // 24
#define F_L_DIR_PIN 23 // 13
#define B_R_DIR_PIN 24 // 19

#define F_STR_PIN 27 // 16
#define B_STR_PIN 28 // 20

#include <rover/DriveCmd.h>
#include <rover/RPM.h>
#include <std_msgs/Empty.h>

float drive_pcnt = 0; // Desired wheel speed percentage
float steer_pcnt = 0; // Desired steering speed percentage

bool alive = false; // True if we have contact with mainframe
int hbeat_cnt = 0; // Counter of how many loops have passed since heartbeat
const float iteration_time = 1/LOOP_HERTZ;

int req_RPM[4]    = {0,0,0,0};
int actual_RPM[4] = {0,0,0,0}; // RPM values as reported by Arduino

// Clamp value within range - convenience function
int clamp(int value, int max, int min)
{
  if (value > max) return max;
  else if (value < min) return min;
  else return value;
}

float fclamp(float value, float max, float min)
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

      /*
      for(int i=0;i<4;i++) 
      {
	  // UNCOMMENT ME WHEN FINISHED TUNING PID
          //req_RPM[i] = abs((drive_pcnt/100)*MAX_RPM);
      } */

      //ROS_INFO_STREAM("Drive command received.");
    }
}


// Callback for subscription to drive command topic "cmd_data"
void encoders_cb(const rover::RPM::ConstPtr& msg)
{    
    actual_RPM[0] = msg->rpm_fl;
    actual_RPM[1] = msg->rpm_fr; 
    actual_RPM[2] = msg->rpm_bl; 
    actual_RPM[3] = msg->rpm_br; 
    
    ROS_INFO_STREAM("RPMs received.");
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

  ros::Subscriber drivecmd_sub = n.subscribe("cmd_data", 5, cmd_data_cb);
  ros::Subscriber encoders_sub = n.subscribe("encoders", 5, encoders_cb);	
  ros::Subscriber hbeat_sub = n.subscribe("hbeat", 1, hbeat_cb);	

  // If no heartbeat for 2 seconds, rover dies
  const int hbeat_timeout = 2*LOOP_HERTZ;

  // ****************** VARIABLES ********************** //

  // Format is [f_l, f_r, b_l, b_r]
  const int dir_pins[4] = {F_L_DIR_PIN, 
                           F_R_DIR_PIN, 
                           B_L_DIR_PIN, 
                           B_R_DIR_PIN};  

  // To correct the motor directions
  const bool correction[4] = {1, 1, 0, 1}; 

  // Translates boolean directions to neg pos directions
  const float dirs[2] = {-1.0, 1.0};
  
  // Skid steering modification
  bool skid_overlay[4] = {1, 1, 1, 1};    

  // Values to reassign the skid overlay for turning on the spot
  const bool skid_dir1[4] = {1, 0, 1, 0};
  const bool skid_dir2[4] = {0, 1, 0, 1};
  const bool skid_orig[4] = {1, 1, 1, 1};

  // PID variables
  /*
  float error[4] = {0,0,0,0};
  float error_prior[4] = {0,0,0,0};
  float integral[4] = {0,0,0,0};
  float derivative[4] = {0,0,0,0};
  float output[4] = {0,0,0,0}; */

  int drive_pwm = 0; // Wheel PWMs when driving normally
  int steer_pwm = 0; // Wheel PWMs when turning on the spot

  bool drive_dir = 1; // Wheel direction
  bool steer_dir = 1; // Steering direction
  bool on_the_spot = 0; //Steering on the spot?

  // Limiters of maximum steering and driving percentages
  float limit_drive = 0;
  float limit_steer = 0;

  // Highest percentage by which steering modifi
  float MAX_STEER_MOD = 0.5;

  /*
  float k_p = 0.0;
  float k_i = 0.0;
  float k_d = 0.0; */

  if (argc == 4)
  {
    limit_drive = atof(argv[1]);
    limit_steer = atof(argv[2]);
    MAX_STEER_MOD = atof(argv[3]);
    /*
    k_p = atof(argv[4]);
    k_i = atof(argv[5]);
    k_d = atof(argv[6]);
    req_RPM[0] = atoi(argv[7]);
    req_RPM[1] = atoi(argv[7]);
    req_RPM[2] = atoi(argv[7]);
    req_RPM[3] = atoi(argv[7]); */
  }

  ROS_INFO_STREAM("max_steer_mod: " << MAX_STEER_MOD);

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

  // ****************** MAIN LOOP *************************** //

  while (ros::ok())
  {
    // If no heartbeat, kill rover
    if (hbeat_cnt > hbeat_timeout) 
    {
   	    alive = false; 
    	ROS_INFO_STREAM("No heartbeat, killing rover :(");   
   	}

    // Set new motor directions
    drive_dir = !(drive_pcnt < 0);
    steer_dir = !(steer_pcnt < 0);

    // If no throttle, switch to "turning on the spot" mode
   	on_the_spot = fabs(drive_pcnt) < 0.2;

	/*
	for(int k=0;k<4;k++) {
	  error[k] = req_RPM[k] - actual_RPM[k];
	  integral[k] = integral[k] + (error[k] * iteration_time);
	  derivative[k] = (error[k] - error_prior[k]) / iteration_time;
	  output[k] = (k_p*error[k]) + (k_i*integral[k]) + (k_d*derivative[k]);
	  error_prior[k] = error[k];

	  drive_pwm[k] = limit_drive*output[k];
	  drive_pwm[k] = clamp(drive_pwm[k], MAX_PWM, 0);
	} */
 

    // If we are turning on the spot (no throttle)
    if (on_the_spot)
    {
	    // Map steering percentage to PWM as a quadratic, with limiter. 
    	steer_pwm = limit_steer*MAX_PWM*pow(steer_pcnt/100, 2);
	
     	// Make sure the PWM val hasn't gone outside range somehow
     	steer_pwm = clamp(steer_pwm, MAX_PWM, -MAX_PWM);

        // Use the steering trigger value as our throttle
	    drive_pwm = steer_pwm;

        // Reassign wheel directions
	    if (steer_dir)
          copy(skid_dir1, skid_dir1+4, skid_overlay);
        else
          copy(skid_dir2, skid_dir2+4, skid_overlay);
    }
    else
    {
        // Map drive percentage to PWM as a quadratic, with limit
	    drive_pwm = limit_drive*MAX_PWM*pow(drive_pcnt/100, 2);

	    // Make sure the PWM val hasn't gone outside range somehow
	    drive_pwm = clamp(drive_pwm, MAX_PWM, 0);

        // Set wheel directions to normal
        copy(skid_orig, skid_orig+4, skid_overlay);
    }

    if (!alive)
    {
      // Stop rover if dead
      drive_pwm = 0;
      steer_pwm = 0;
    }

    ROS_INFO_STREAM("on the spot: " << on_the_spot);

    for (int i = 0; i < 4; i++)
    {
      // Amount to steer by, if not steering on the spot
      float steer_mod = 1;

      if (!on_the_spot)
      {
        // Add and subtract steering proportion from left and right wheels according to
        //  steering direction. This is inverted if driving backwards.
        if (i == 0 || i == 2)
            steer_mod = 1 - dirs[drive_dir]*dirs[steer_dir]*fclamp(fabs(steer_pcnt/100), 
                MAX_STEER_MOD, 0);
        else 
            steer_mod = 1 + dirs[drive_dir]*dirs[steer_dir]*fclamp(fabs(steer_pcnt/100), 
                MAX_STEER_MOD, 0);
      }          

      // Correct the wheel directions
      bool direction = (correction[i] != drive_dir);

      // If skid steering command requires opposite direction, switch	
      if (skid_overlay[i] == 0) direction = !direction;

      digitalWrite (dir_pins[i], direction);

      // Change wheel speed outputs
      pwmWrite(PIN_BASE + i, steer_mod*drive_pwm); // pins of PWM board, (0, 1, 2, 3)
    }

    hbeat_cnt++; // Increment heartbeat tracking timer

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
