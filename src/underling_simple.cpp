/****************************************************************************************
 *  NOVA ROVER TEAM - URC2018
 *  This is the code executed onboard the rover to manage all driving-related aspects,
 *  such as motor speed control, motor direction control, etc.
 *  
 *  Link to motors:
 *  https://www.servocity.com/118-rpm-hd-premium-planetary-gear-motor-w-encoder
 *  
 *  Link to PWM board:
 *  http://www.robotshop.com/uk/pca9685-16-channel-12-bit-pwm-servo-driver.html
 * 
 *  Author: Ben Steer
 *  Last modified by: Ben Steer (16/02/2018)
 ****************************************************************************************/

/***************************************************************************************************
* INCLUDES, DECLARATIONS AND GLOBAL VARIABLES
***************************************************************************************************/
// ________________________General includes________________________
#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm> // ?
#include <iostream>
#include <wiringPi.h>
#include "pca9685/src/pca9685.h" // PWM board library
using namespace std;

// __________________________ROS includes__________________________
#include <rover/DriveCmd.h>
#include <rover/RPM.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

// __________________________Definitions___________________________
#define LOOP_HERTZ 50 // Main control loop rate

#define K_P 1.2
#define K_I 5.0
#define K_D 0.025

#define MIN_VOLTAGE 10.0

// PWM/RPM Definitions
#define PIN_BASE 160
#define PWM_HERTZ 1000
#define MAX_PWM 4096 // Max PWM of pca9685
#define MAX_RPM 128 // Experimentally obtained max RPM

// Pin definitions for direction-changing GPIOs
#define B_L_DIR_PIN 4 // 23
#define F_R_DIR_PIN 5 // 24
#define F_L_DIR_PIN 23 // 13
#define B_R_DIR_PIN 24 // 19
#define F_STR_PIN 27 // 16
#define B_STR_PIN 28 // 20

// Global variables
float limit_drive = 0;
float limit_steer = 0;
float drive_pcnt = 0;	// Desired wheel speed percentage
float steer_pcnt = 0;	// Desired steering speed percentage
<<<<<<< HEAD
bool alive = false;		// True if we have contact with mainframe
=======
int tempCount = 0;
int tempCount2 = 0;

bool alive = false;		// False means rover ceases movement, either radio loss or low battery
bool hbeat = false;   // Do we have a heartbeat?
bool volt_ok = false; // Is voltage level okay?

>>>>>>> 86556846ec8ee7186ac9a79ae78eae1097f5ce70
int hbeat_cnt = 0;		// Counter of how many loops have passed since heartbeat

const float iteration_time = 1.0/LOOP_HERTZ;	// Iteration time of the main loop

int req_RPM[4] = {0,0,0,0};		// The RPM values which are desired for each wheel
int actual_RPM[4] = {0,0,0,0}; 	// The RPM values for each wheel as reported by the Arduino
int steer_mod[4] = {0,1,1,0};
float MAX_STEER_MOD = 0.5;

/***************************************************************************************************
* CLAMP FUNCTION
*
* A convenience function - this function clamps a value within a certain range, dependent on
* the passed minimum and maximum argument values.
*
* Inputs:	int value - The value to be clamped
*			int max - The maximum value this value may be
*			int min - The minimum value this value may be
*
* Output:	Int of either value, min or max, depending on value.
* todo: change arguments from (...,max,min) to (...,min,max) for sanity and modify code appropriately
***************************************************************************************************/
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

int MapRPMToPWM(float RPM)
{
  int PWM;
  if(RPM > 0) 
  {
    PWM = round((RPM + 3.744653)/0.03281);	// Determined by plotting PWM vs. RPM and obtaining the line of best fit
    PWM = clamp(PWM, MAX_PWM, 0);	// Clamp PWM to valid value
  }
  else PWM = 0;
  return PWM;
}

/***************************************************************************************************
* ROLLOVER FUNCTION
*
* A convenience function - this function will "rollover" a value if it exceeds a limit so that it
* will stay within a certain range.
*
* Inputs:	int value - The value to be rolled over
* 			int max - The maximum rollover limit
*			int min - The minimum rollover limit
*
* Output:	Int of either value, min or max, depending on value.
* todo: change arguments from (...,max,min) to (...,min,max) for sanity and modify code appropriately
***************************************************************************************************/
int rollover(int value, int max, int min)
{
  if (value > max) return min;
  else if (value < min) return max;
  else return value;
}

/***************************************************************************************************
* COMMAND CALLBACK FUNCTION
*
* The callback function for the subscription to drive command topic "cmd_data". This function is 
* called whenever new "cmd_data" data is published and sets the new acceleration and 
* steering percentage for the rover via global variables.
*
* Input:	const rover::DriveCmd::ConstPtr& msg) - The message object containing the relevent data
***************************************************************************************************/
void cmd_data_cb(const rover::DriveCmd::ConstPtr& msg)
{    
    int speedL, speedR;
    if (alive)
    {
      drive_pcnt = msg->acc;              
      steer_pcnt = 100.0*(msg->steer)/45; // Store desired steering angle as %
      if(fabs(drive_pcnt) < 0.2) {
        if(steer_pcnt<0) {
          steer_mod[0] = 0;
          steer_mod[1] = 1;
          steer_mod[2] = 1;
          steer_mod[3] = 1;
        }
        else { 
          steer_mod[0] = 1;
          steer_mod[1] = 0;
          steer_mod[2] = 0;
          steer_mod[3] = 0;
        }
        speedL = limit_drive*fabs((steer_pcnt*MAX_RPM)/100);      
        req_RPM[0] = speedL;
        req_RPM[1] = speedL;
        req_RPM[2] = speedL;
        req_RPM[3] = speedL;
      }
      else {
        steer_mod[0] = 1;
        steer_mod[1] = 1;
        steer_mod[2] = 0;
        steer_mod[3] = 1;
        if(steer_pcnt>0) {
          speedL = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)+(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100)); 
          speedR = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)-(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100));
		}
        else {
          speedL = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)-(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100)); 
          speedR = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)+(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100));
        }     
        req_RPM[0] = speedR;
        req_RPM[1] = speedL;
        req_RPM[2] = speedR;
        req_RPM[3] = speedL;
      }
    }
}

/***************************************************************************************************
* ENCODERS CALLBACK FUNCTION
*
* The callback function for the subscription to the encoder data. This function is 
* called whenever new RPM calculations are sent by the Arduino and saves the RPM values into the
* global array actual_RPM so the data may be used in the motor PID controller.
*
* Input:	const rover::DriveCmd::ConstPtr& msg - The message object containing the relevent data.
***************************************************************************************************/
void encoders_cb(const rover::RPM::ConstPtr& msg)
{   
    // Record data into array index respective to each wheel
    actual_RPM[0] = msg->rpm_br;
    actual_RPM[1] = msg->rpm_bl;
	// encoders not working
    //actual_RPM[2] = msg->rpm_fr; 
	//actual_RPM[3] = msg->rpm_fl;
    actual_RPM[2] = msg->rpm_br; 
    actual_RPM[3] = msg->rpm_bl;
}

/***************************************************************************************************
* HEARTBEAT CALLBACK FUNCTION
*
* The callback function for the subscription to the heartbeat from the mainframe. This function is 
* called whenever a new heartbeat is sent from the mainframe.
*
* Whenever a new heartbeat is received from the mainframe, this means that the rover can still communicate
* with the mainframe. The mainframe can still
* Input:	const rover::DriveCmd::ConstPtr& msg - The message object containing the relevent data.
***************************************************************************************************/
void hbeat_cb(const std_msgs::Empty::ConstPtr& msg)
{
  hbeat = true;
  hbeat_cnt = 0;
}

/***************************************************************************************************
* VOLTAGE CALLBACK FUNCTION
*
* The callback function for the subscription to the voltage from the Arduino Pro Micro.
*
***************************************************************************************************/
void voltage_cb(const std_msgs::Float32::ConstPtr& msg)
{
  if (msg->data < MIN_VOLTAGE)
  {
    volt_ok = false;
    ROS_INFO_STREAM("DANGEROUS VOLTAGE LEVEL REACHED ** BEE-BAH **");   
  }
  else
    volt_ok = true;
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
  ros::Subscriber voltage_sub = n.subscribe("voltage", 1, voltage_cb);	

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

  // PID variables
  
  float error[4] = {0,0,0,0};
  float error_prior[4] = {0,0,0,0};
  float integral[4] = {0,0,0,0};
  float derivative[4] = {0,0,0,0};
  float output[4] = {0,0,0,0};

  int drive_pwm[4] = {0,0,0,0};


  bool drive_dir = 1; // Wheel direction
  bool steer_dir = 0.5; // Steering direction
  bool on_the_spot = 0; //Steering on the spot?

  // Highest percentage by which steering is modified


  if (argc == 4)
  {
    limit_drive = atof(argv[1]);
    limit_steer = atof(argv[2]);
    MAX_STEER_MOD = atof(argv[3]);
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
  ros::Duration(5).sleep();
  // ****************** MAIN LOOP *************************** //

  while (ros::ok())
  {
<<<<<<< HEAD
    // If no heartbeat, kill rover
    if (hbeat_cnt > hbeat_timeout) 
    {
   	    alive = false; 
   	}
=======
    // Check heartbeat, voltage levels and decide whether to kill the rover 
    if (~hbeat || ~volt_ok)
      alive = false;
    else
      alive = true;
>>>>>>> 86556846ec8ee7186ac9a79ae78eae1097f5ce70

    // Set new motor directions
    drive_dir = !(drive_pcnt < 0);
    steer_dir = !(steer_pcnt < 0);

    for(int k=0;k<4;k++) 
    {
        error[k] = req_RPM[k] - actual_RPM[k];
	    integral[k] = integral[k] + (error[k] * iteration_time);
	    derivative[k] = (error[k] - error_prior[k]) / iteration_time;
	    output[k] = (K_P*error[k]) + (K_I*integral[k]) + (K_D*derivative[k]);
	    error_prior[k] = error[k];
        integral[k] = fclamp(integral[k], 25.0, -25.0);

        drive_pwm[k] = MapRPMToPWM(round(actual_RPM[k]+output[k]));
    }
<<<<<<< HEAD
=======
    //drive_pwm[3] = MapRPMToPWM(req_RPM[3]);
	

    if(tempCount==3) {
        //ROS_INFO_STREAM("INCREASING PWM");
        
        ROS_INFO_STREAM("Req RPM: " << req_RPM[0] << " Actual RPM: " << actual_RPM[0] << " Drive PWM: " << drive_pwm[0] << " Output: " << output[0] << " Integral: " << integral[0] << " Derivative: " << derivative[0] << " Error: " << error[0] << " iterationTime: " << iteration_time);
>>>>>>> 86556846ec8ee7186ac9a79ae78eae1097f5ce70

	
    if (!alive)
    {
      // Stop rover if dead
      drive_pwm[0] = 0;
      drive_pwm[1] = 0;
      drive_pwm[2] = 0;
      drive_pwm[3] = 0;
    }

    for (int i = 0; i < 4; i++)
    {
      bool direction = (correction[i] != drive_dir);
      direction = (steer_mod[i] != drive_dir);

      // If skid steering command requires opposite direction, switch	
      if (skid_overlay[i] == 0) direction = !direction;

      digitalWrite (dir_pins[i], direction);
      // Change wheel speed outputs
      pwmWrite(PIN_BASE + i, drive_pwm[i]); // pins of PWM board, (0, 1, 2, 3)
    }
<<<<<<< HEAD
=======
    tempCount++;

      // If no heartbeat, kill rover
    if (hbeat_cnt > hbeat_timeout) 
    {
      hbeat = false;
    	//ROS_INFO_STREAM("No heartbeat, killing rover :(");   
   	}
>>>>>>> 86556846ec8ee7186ac9a79ae78eae1097f5ce70

    hbeat_cnt++; // Increment heartbeat tracking timer

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
