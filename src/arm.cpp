#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 1000

#define INCR 10

#include "gamepad/gamepad.h"

int clamp(int pwm, int max, int min)
{
  int result = pwm;

  if (pwm > max)
    result = max;
  else if (pwm < min)
    result = min;

  return result;
  
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
  if (fd < 0)
  {
    printf("Error in setup\n");
    return fd;
  }

  pca9685PWMReset(fd); // Reset all output

  GamepadInit();

  wiringPiSetup();

  int pwm[6] = {0, 0, 0, 0, 0, 0};
  int dir[6] = {1, 1, 1, 1, 1, 1};

  int correct[6] = {1, 1, 1, 1, 1, 1}; // Fix dir errors with this

  while (ros::ok())
  {
    GamepadUpdate();

    /*
      Degree    Control
      ----------------------------------
      1         right stick horizontal
      2         right stick vertical
      3         left bumper + right stick vertical
      4         left stick vertical
      5         left stick horizontal
      6         triggers
    */

    int l_stick_x = 0;
    int l_stick_y = 0;
    int r_stick_x = 0;
    int r_stick_y = 0;

    // Get raw left/right stick values
    GamepadStickXY(GAMEPAD_0, STICK_LEFT, &l_stick_x, &l_stick_y);
    GamepadStickXY(GAMEPAD_0, STICK_RIGHT, &r_stick_x, &r_stick_y);

    // Normalised trigger values
    float r_trigger = GamepadTriggerValue(GAMEPAD_0, TRIGGER_RIGHT);
    float l_trigger = GamepadTriggerValue(GAMEPAD_0, TRIGGER_LEFT);
  
    // Get left bumper value
    bool l_bump = GamepadButtonDown(GAMEPAD_0, BUTTON_LEFT_SHOULDER);

    // Normalise the stick values
    float fl_stick_x = ((float) l_stick_x)/32767;
    float fl_stick_y = ((float) l_stick_y)/32767;
    float fr_stick_x = ((float) r_stick_x)/32767;
    float fr_stick_y = ((float) r_stick_y)/32767;


    // *************** Set directions based on inputs
    if (fr_stick_x > 0) dir[0] = 1;
    else dir[0] = -1;

    if (fr_stick_y > 0) {
      dir[1] = 1;
      dir[2] = 1;
    }
    else {
      dir[1] = -1;
      dir[2] = -1;
    }

    if (fl_stick_y > 0) dir[3] = 1;
    else dir[3] = -1;

    if (fl_stick_x > 0) dir[4] = 1;
    else dir[4] = -1;

    if (r_trigger > 0) dir[5] = 1;
    else dir[5] = -1;
    // **************** Done setting directions
     

    // **************** Increment PWM values
    pwm[0] += fabs(fr_stick_x)*correct[0]*dir[0]*((float)INCR);

    if (!l_bump) 
      pwm[1] += fabs(fr_stick_y)*correct[1]*dir[1]*((float)INCR);
    else
      pwm[2] += fabs(fr_stick_y)*correct[2]*dir[2]*((float)INCR);

    pwm[3] += fabs(fl_stick_y)*correct[3]*dir[3]*((float)INCR);
    pwm[4] += fabs(fl_stick_x)*correct[4]*dir[4]*((float)INCR);

    if (dir[5] == 1) // If right trigger
      pwm[5] += fabs(r_trigger)*correct[5]*dir[5]*((float)INCR);
    else
      pwm[5] += fabs(l_trigger)*correct[5]*dir[5]*((float)INCR);
    // ***************** Done incrementing PWM values
    

    // Clamp PWM values and write
    for (int i=0; i<6; i++)     
    { 
      pwm[i] = clamp(pwm[i], MAX_PWM, 0);
      pwmWrite(PIN_BASE + i, pwm[i]);
    }   

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
 
  /* ------ FOR 50Hz SERVOS --------
  #define MAX_PULSE 600
  #define MIN_PULSE 150
  */

  //#define THRES 0.5

  /*--------- FIRST TEST, SERVO SWEEP -------------
  bool increasing = true;
  int value = 200;
  */

  /* --------- SECOND TEST, ROBOT ARM CONTROLLER -------------
  GamepadInit(); // Initialise the Xbox gamepad
  int mid = MIN_PULSE + (MAX_PULSE - MIN_PULSE);
  int pulse_12 = mid;
  int pulse_13 = mid;
  int pulse_14 = mid;
  int pulse_15 = mid;
  */

    /* --------- SECOND TEST, ROBOT ARM CONTROLLER -------------
    GamepadUpdate();

    int l_stick_x = 0;
    int l_stick_y = 0;
    int r_stick_x = 0;
    int r_stick_y = 0;    

    GamepadStickXY(GAMEPAD_0, STICK_LEFT, &l_stick_x, &l_stick_y);
    GamepadStickXY(GAMEPAD_0, STICK_RIGHT, &r_stick_x, &r_stick_y);

    float lf_stick_x = -((float) l_stick_x)/32767;
    float lf_stick_y = -((float) l_stick_y)/32767;
    float rf_stick_y = ((float) r_stick_y)/32767;

    if (lf_stick_x > THRES) // Base
      pulse_12 += 5;
    else if (lf_stick_x < -THRES)
      pulse_12 -= 5;

    if (lf_stick_y > THRES) // Shoulder
      pulse_13 += 5;
    else if (lf_stick_y < -THRES)
      pulse_13 -= 5;

    if (rf_stick_y > THRES) // Elbow
      pulse_14 += 5;
    else if (rf_stick_y < -THRES)
      pulse_14 -= 5;

    bool r_trigger = GamepadTriggerDown(GAMEPAD_0, TRIGGER_RIGHT);
    bool l_trigger = GamepadTriggerDown(GAMEPAD_0, TRIGGER_LEFT);

    if (r_trigger == true) // Wrist
      pulse_15 += 5;
    else if (l_trigger == true)
      pulse_15 -= 5;

    if (pulse_15 < MIN_PULSE)
      pulse_15 = MIN_PULSE;
    else if (pulse_15 > MAX_PULSE)
      pulse_15 = MAX_PULSE;

    if (pulse_12 < MIN_PULSE)
      pulse_12 = MIN_PULSE;
    else if (pulse_12 > MAX_PULSE)
      pulse_12 = MAX_PULSE;

    if (pulse_13 < MIN_PULSE)
      pulse_13 = MIN_PULSE;
    else if (pulse_13 > MAX_PULSE)
      pulse_13 = MAX_PULSE;

    if (pulse_14 < MIN_PULSE)
      pulse_14 = MIN_PULSE;
    else if (pulse_14 > MAX_PULSE)
      pulse_14 = MAX_PULSE;
		
    pwmWrite(PIN_BASE + 12, pulse_12);
    pwmWrite(PIN_BASE + 13, pulse_13);
    pwmWrite(PIN_BASE + 14, pulse_14);
    pwmWrite(PIN_BASE + 15, pulse_15);
    */

    /*--------- FIRST TEST, SERVO SWEEP -------------
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

