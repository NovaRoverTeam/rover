#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <iostream>
using namespace std;

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 10

#define F_L_DIR_PIN 24 // TODO set these GPIOs
#define F_R_DIR_PIN 25
#define B_L_DIR_PIN 26
#define B_R_DIR_PIN 27

#define DRV_INCR 50
#define STR_INCR 50

#include <rover/DriveCommand.h>
#include <rover/SteerCommand.h>

static volatile int enc_count = 0;
static volatile float ang_vel = 0;

// Format is [f_l, f_r, b_l, b_r]

int dir_pins[4] = {F_L_DIR_PIN, F_R_DIR_PIN, B_L_DIR_PIN, B_R_DIR_PIN};

int dir[4] = {1}; // Direction of each motor
int pwm[4] = {0}; // PWM speed value for each motor
int pwm_des[4] = {0}; // Desired PWM speed values

int steer_dir = 0;
int steer_pwm_f = MAX_PWM/2;
int steer_pwm_b = MAX_PWM/2;

// TODO RANGE CHECKS *****TODO*****TODO******************

bool drive_cb(rover::DriveCommand::Request  &req,
         rover::DriveCommand::Response &res)
{
  pwm_des[0] = req.f_wheel_l; // Grab wheel PWMs
  pwm_des[1] = req.f_wheel_r;
  pwm_des[2] = req.b_wheel_l;
  pwm_des[3] = req.b_wheel_r;

  //cout << "SERVICE HAS BEEN CALLED - WOWZERS" << endl;
  //cout << pwm_des[0] << " " << pwm_des[1] << " " << pwm_des[2] << " " << pwm_des[3] << endl;

  return true;
}

bool steer_cb(rover::SteerCommand::Request  &req,
         rover::SteerCommand::Response &res)
{
  bool steer_left = req.steer_left; // Grab steer direction

  if (steer_left) steer_dir = -1;
  else steer_dir = 1;

  cout << "STEER SERVICE HAS BEEN CALLED - WOWZERS" << endl;
  //cout << pwm_des[0] << " " << pwm_des[1] << " " << pwm_des[2] << " " << pwm_des[3] << endl;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "underling");
  ros::NodeHandle n;
  ros::Rate loop_rate(HERTZ);

  ros::ServiceServer drv_service = n.advertiseService("/DriveCommand", drive_cb);	
  ros::ServiceServer str_service = n.advertiseService("/SteerCommand", steer_cb);	


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
    //pinMode (dir_pins[i], OUTPUT);
    //digitalWrite (dir_pins[i], dir[i]);
  }

  while (ros::ok())
  {
    int pwm_dif[4] = {0, 0, 0, 0}; 
    int dir_tmp[4] = {1, 1, 1, 1};
    int incr[4] = {DRV_INCR, DRV_INCR, DRV_INCR, DRV_INCR};

    for (int i = 0; i < 4; i++)
    {
      cout << "incr " << i << " is " << incr[i] << endl;

      pwm_dif[i] = pwm_des[i] - pwm[i]; // Diff between cur and des
      
      if (pwm_dif[i] < 0) dir_tmp[i] = -1; // Dir to DRV_INCR in
      if (abs(pwm_dif[i]) < 2*DRV_INCR) incr[i] = 1; // How much change

      pwm[i] = pwm[i] + dir_tmp[i]*incr[i]; // Move pwm vals closer to des

      if (pwm[i] < 0) dir[i] = -1; // Set new motor directions
      else dir[i] = 1;

      // ***************** CHANGE OUTPUTS *********************
      //digitalWrite (dir_pins[i], dir[i]);
      //pwmWrite(PIN_BASE + i, pwm[i]); // pins of pwm board, (0, 1, 2, 3)
      cout << "drv dir " << i << " is " << dir[i] << endl;
      cout << "drv pwm " << i << " is " << pwm[i] << endl << endl;
    }

    cout << "str dir is " << steer_dir << endl;

    if (steer_dir != 0) // Adjust steering if necessary
    {
      steer_pwm_f += steer_dir*STR_INCR;
      steer_pwm_b -= steer_dir*STR_INCR;

      steer_dir = 0; // Reset
    }

    // Validate PWM values
    if (steer_pwm_f > MAX_PWM) steer_pwm_f = MAX_PWM;
    else if (steer_pwm_f < 0) steer_pwm_f = 0;
    if (steer_pwm_b > MAX_PWM) steer_pwm_b = MAX_PWM;
    else if (steer_pwm_b < 0) steer_pwm_b = 0;

    //pwmWrite(PIN_BASE + 5, steer_pwm_f); // pins of pwm board, (5, 6 for steering)
    //pwmWrite(PIN_BASE + 6, steer_pwm_b);

    cout << "str pwm front is " << steer_pwm_f << endl << endl;
    cout << "str pwm back is " << steer_pwm_b << endl << endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


