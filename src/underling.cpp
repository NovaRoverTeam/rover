#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <iostream>
using namespace std;

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 1000

#define F_L_DIR_PIN 24 // TODO set these GPIOs
#define F_R_DIR_PIN 25
#define B_L_DIR_PIN 26
#define B_R_DIR_PIN 27

#define INCREMENT 5

#include <rover/DriveCommand.h>

static volatile int enc_count = 0;
static volatile float ang_vel = 0;

// Format is [f_l, f_r, b_l, b_r]

int dir_pins[4] = {F_L_DIR_PIN, F_R_DIR_PIN, B_L_DIR_PIN, B_R_DIR_PIN};

int dir[4] = {1}; // Direction of each motor
int pwm[4] = {0}; // PWM speed value for each motor
int pwm_des[4] = {0}; // Desired PWM speed values

bool drive_cb(rover::DriveCommand::Request  &req,
         rover::DriveCommand::Response &res)
{
  pwm_des[0] = req.f_wheel_l; // Grab wheel PWMs
  pwm_des[1] = req.f_wheel_r;
  pwm_des[2] = req.b_wheel_l;
  pwm_des[3] = req.b_wheel_r;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underling");
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
    //pinMode (dir_pins[i], OUTPUT);
    //digitalWrite (dir_pins[i], dir[i]);
  }

  while (ros::ok())
  {
    int pwm_dif[4] = {0}; 
    int dir_tmp[4] = {1};
    int incr[4] = {INCREMENT};

    for (int i = 0; i < 4; i++)
    {
      pwm_dif[i] = pwm_des[i] - pwm[i]; // Diff between cur and des

      if (pwm_dif[i] < 0) dir_tmp[i] = -1; // Dir to increment in

      if (abs(pwm_dif[i]) < 2*INCREMENT) incr[i] = 1; // How much change

      pwm[i] = dir_tmp[i]*incr[i]; // Move pwm vals closer to des

      if (pwm[i] < 0) dir[i] = -1; // Set new motor directions
      else dir[i] = 1;

      // ***************** CHANGE OUTPUTS *********************
      //digitalWrite (dir_pins[i], dir[i]);
      //pwmWrite(PIN_BASE + i, pwm[i]); // pins of pwm board, (0, 1, 2, 3)
      cout << "direction " << i << " is " << dir[i] << endl;
      cout << "pwm " << i << " is " << pwm[i] << endl << endl;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

