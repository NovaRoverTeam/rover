#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>

#include <wiringPi.h>
#include <pca9685.h>

#define PIN_BASE 160
#define MAX_PWM 4096
#define HERTZ 1000

#define DIR_PIN 24

#define ENC_PIN //TODO
#define ENC_HZ 10

static volatile int enc_count = 0;
static volatile float ang_vel = 0;

#include "gamepad/gamepad.h"

void encISR() 
{
  ++enc_count;
}

PI_THREAD (encThread)
{
  wiringPiISR(ENC_PIN, INT_EDGE_RISING, &encISR);

  float del_time = 1000.0/((float)ENC_HZ);
  float two_pi = 2.0*M_PI;

  for (;;)
  {
    enc_count = 0;
    delay(del_time); // Delay in ms between readings

    ang_vel = two_pi*(((float)enc_count)/24.0)/del_time;
    ROS_INFO("Angular velocity: %.2f rad/s.", ang_vel);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "underling");
  ros::NodeHandle n;
  ros::Rate loop_rate(HERTZ); // 1000 Hz

  int PWM_val = 0;
  bool dir = HIGH;

  int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
  if (fd < 0)
  {
    printf("Error in setup\n");
    return fd;
  }

  pca9685PWMReset(fd); // Reset all output

  GamepadInit();

  piThreadCreate (encThread); // Start encoders thread

  wiringPiSetup();
  pinMode (DIR_PIN, OUTPUT);
  digitalWrite (DIR_PIN, dir);

  while (ros::ok())
  {
    GamepadUpdate();

    bool r_trigger = GamepadTriggerDown(GAMEPAD_0, TRIGGER_RIGHT);
    bool l_trigger = GamepadTriggerDown(GAMEPAD_0, TRIGGER_LEFT);
  
    bool a_but = GamepadButtonTriggered(GAMEPAD_0, BUTTON_A);

    if (r_trigger == true)
      PWM_val += 1;
    else if (l_trigger == true)
      PWM_val -= 1;

    if (PWM_val < 0) PWM_val = 0;
    else if (PWM_val > MAX_PWM) PWM_val = MAX_PWM;

    if ((a_but == true) && (PWM_val == 0))
    {
      dir = !dir;
      digitalWrite (DIR_PIN, dir);
    }

    //ROS_INFO_STREAM(PWM_val);

    pwmWrite(PIN_BASE + 15, PWM_val);

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

