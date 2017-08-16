#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <iostream>
using namespace std;

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 2

#define B_L_DIR_PIN 4 // 23
#define F_R_DIR_PIN 5 // 24
#define F_L_DIR_PIN 23 // 13
#define B_R_DIR_PIN 24 // 19

#define F_STR_PIN 27 // 16
#define B_STR_PIN 28 // 20

#define DRV_INCR 100

#include <rover/DriveCommand.h>
#include <rover/SteerCommand.h>

static volatile int enc_count = 0;
static volatile float ang_vel = 0;

// Format is [f_l, f_r, b_l, b_r]

int dir_pins[4] = {F_L_DIR_PIN, F_R_DIR_PIN, B_L_DIR_PIN, B_R_DIR_PIN};

int dir[4] = {1, 1, 1, 1}; // Direction of each motor
int pwm[4] = {0, 0, 0, 0}; // PWM speed value for each motor
int pwm_des[4] = {1000, 1000, 1000, 1000}; // Desired PWM speed values
int correction[4] = {0, 1, 0, 1}; // Correct motor directions

bool do_steer = true;
bool steer_dir = 0;
int steer_pwm = MAX_PWM/4;
int single = 1; // 1 means not single, 0 means single

// TODO RANGE CHECKS *****TODO*****TODO******************

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug");
  ros::NodeHandle n;
  ros::Rate loop_rate(HERTZ);

  int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
  if (fd < 0)
  {
    printf("Error in setup\n");
    return fd;
  }

  pca9685PWMReset(fd); // Reset all output

  wiringPiSetup();

  for (int i = 0; i < 4; i++) // Configure GPIOs and set init dirs
  {
    pinMode (dir_pins[i], OUTPUT);
    digitalWrite (dir_pins[i], correction[i]*dir[i]);
  }

  pinMode (F_STR_PIN, OUTPUT);
  pinMode (B_STR_PIN, OUTPUT);
  digitalWrite (B_STR_PIN, steer_dir);
  digitalWrite (F_STR_PIN, !steer_dir);

  //steer_dir = stoi(argv[1]);
  //single = stoi(argv[2]);

  while (ros::ok())
  { 
    /*
    int pwm_dif[4] = {0, 0, 0, 0}; 
    int dir_tmp[4] = {1, 1, 1, 1};
    int incr[4] = {DRV_INCR, DRV_INCR, DRV_INCR, DRV_INCR};

    for (int i = 0; i < 4; i++)
    {
      pwm_dif[i] = pwm_des[i] - pwm[i]; // Diff between cur and des
      
      if (pwm_dif[i] < 0) dir_tmp[i] = -1; // Dir to DRV_INCR in
      if (abs(pwm_dif[i]) < 2*DRV_INCR) incr[i] = 1; // How much change

      pwm[i] = pwm[i] + dir_tmp[i]*incr[i]; // Move pwm vals closer to des

      if (pwm[i] < 0) dir[i] = -1; // Set new motor directions
      else dir[i] = 1;

      // ***************** CHANGE OUTPUTS *********************
      digitalWrite (dir_pins[i], correction[i]*dir[i]);
      pwmWrite(PIN_BASE + i, abs(pwm[i])); // pins of pwm board, (0, 1, 2, 3)
      cout << "drv dir " << i << " is " << correction[i]*dir[i] << endl;
      cout << "drv pwm " << i << " is " << abs(pwm[i]) << endl << endl;
    }*/
    
    
    //if (do_steer) // Adjust steering if necessary
    //{
      
      int var;
      cin >> var;

      switch(var) 
      {
        case 0:
	{ // STOP EVERYTHING
	  for (int i = 0; i < 6; i++) pwmWrite(PIN_BASE + i, 0);
	  break;
	} 
        case 1: 
	{
	  for (int i = 0; i < 4; i++) pwmWrite(PIN_BASE + i, MAX_PWM/4);
	  break;
	}
	case 2: 
        { 
          for (int i = 0; i < 4; i++) pwmWrite(PIN_BASE + i, MAX_PWM/2);
          break;
	}
	case 3: 
        { 
          for (int i = 0; i < 4; i++) pwmWrite(PIN_BASE + i, 3*MAX_PWM/4);
          break;
	}
	case 4: 
        { 
          for (int i = 0; i < 4; i++) pwmWrite(PIN_BASE + i, MAX_PWM);
	  break;
        }
	case 6:
	{
	  single = !single;
	  break;
	}
	case 7: 
        { 
	  pwmWrite(PIN_BASE + 4, MAX_PWM/4);
	  pwmWrite(PIN_BASE + 5, single*MAX_PWM/4);
          digitalWrite (B_STR_PIN, steer_dir);
          digitalWrite (F_STR_PIN, !steer_dir);
	  break;
        }
        case 8:
	{ 
          pwmWrite(PIN_BASE + 4, MAX_PWM/4);
          pwmWrite(PIN_BASE + 5, single*MAX_PWM/4);
          digitalWrite (B_STR_PIN, !steer_dir);
          digitalWrite (F_STR_PIN, steer_dir);
	  break;
        }
	case 9:
	{
	  for (int i = 0; i < 4; i++) 
	  {
	    correction[i] = !correction[i];
	    digitalWrite (dir_pins[i], correction[i]*dir[i]);
	  }	
	  break;
        }
	default:
	{
	  for (int i = 0; i < 6; i++) pwmWrite(PIN_BASE + i, 0);
	  break;
	}
      } 

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


