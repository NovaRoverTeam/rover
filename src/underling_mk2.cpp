/****************************************************************************************
 *  NOVA ROVER TEAM - URC2018
 *  This is the code executed onboard the rover to manage all driving-related aspects,
 *  such as motor speed control, motor direction control, etc.
 *  
 *  Motor model:
 *  Midwest Motion Products MMP S22-346E-24V GP52-059
 *  
 *  Link to PWM board:
 *  http://www.robotshop.com/uk/pca9685-16-channel-12-bit-pwm-servo-driver.html
 * 
 *  Authors: Steerzy & Bruno
 ****************************************************************************************/

/***************************************************************************************************
* INCLUDES, DECLARATIONS AND GLOBAL VARIABLES
***************************************************************************************************/
// ________________________General includes________________________
#include "ros/ros.h"
#include <ros/console.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <string>
#include "pca9685/src/pca9685.h" // PWM board library
#include <signal.h>
using namespace std;

// __________________________ROS includes__________________________
#include <rover/DriveCmd.h>
#include <rover/ArmCmd.h>
#include <rover/RPM.h>
#include <std_msgs/Float32.h>
#include <rover/ReqRPM.h>
#include <rover/Ball.h>

// __________________________Definitions___________________________
#define LOOP_HERTZ              50 // Main control loop rate
#define ON_THE_SPOT_THRESHOLD   0.2 // Highest drive percentage value before
#define BALL_CAMERA_WIDTH       640
#define BALL_TRACK_MIDDLE_WIDTH 50
#define BALL_MAX_RADIUS         50
#define BALL_TRACK_SPEED        20

// PID K parameters
#define K_P     1
#define K_I     2
#define K_D     0.01

// PWM/RPM Definitions
#define PIN_BASE    160
#define PWM_HERTZ   1000
#define MAX_PWM     4096 // Max PWM of pca9685
#define MAX_RPM     128  // Legacy? Max motor RPMs should be lower

// Pin definitions for direction-changing GPIOs
#define B_L_DIR_PIN 4   // 23
#define F_R_DIR_PIN 5   // 24
#define F_L_DIR_PIN 23  // 13
#define B_R_DIR_PIN 24  // 19
#define RELAY_1_PIN 22  // 6
#define RELAY_2_PIN 25  // 26
#define ZERO_RST_PIN 29  // 21

#define NORMAL_DRIVE        0
#define ON_THE_SPOT_LEFT    1
#define ON_THE_SPOT_RIGHT   2

// Global variables
float limit_drive   = 0;
float limit_steer   = 0;
float drive_pcnt    = 0;    // Desired wheel speed percentage
float steer_pcnt    = 0;    // Desired steering speed percentage


const float iteration_time = 1.0/LOOP_HERTZ;    // Iteration time of the main loop

int desired_RPM[4]  = {0,0,0,0};    // The RPM values which are desired for each wheel
int actual_RPM[4]   = {0,0,0,0};    // The RPM values for each wheel as reported by the Arduino
int steer_mod[4]    = {1,0,0,1};    // The rotation direction of each wheel
int drive_pwm[4]    = {0,0,0,0};    // The PWM values which are output to the motor controllers
int ball_tracking_enabled = 0;
int ball_x = -1;
int ball_radius = -1;

int arm_fd; // File descriptor for arm serial connection
string prev_state; // Keep track of previous state for arm Arduino reset

ros::NodeHandle* n;

/***************************************************************************************************
* CLAMP FUNCTION
*
* A convenience function - this function clamps a value within a certain range, dependent on
* the passed minimum and maximum argument values.
*
* Inputs:   int value - The value to be clamped
*           int max - The maximum value this value may be
*           int min - The minimum value this value may be
*
* Output:   Int of either value, min or max, depending on value.
* todo: change arguments from (...,max,min) to (...,min,max) for sanity and modify code appropriately
***************************************************************************************************/
int clamp(int value, int max, int min)
{
    if (value > max) return max;
    else if (value < min) return min;
    else return value;
}

// Check clamp function description for details - the functions are identical, except this function
// is used for float values
float fclamp(float value, float max, float min)
{
    if (value > max) return max;
    else if (value < min) return min;
    else return value;
}

/***************************************************************************************************
* MAP RPM TO PWM FUNCTION
*
* This function maps the desired RPM value for a wheel to a PWM value and returns it. To obtain the
* mapping function, measure the RPM output of the wheels for various PWM values. (Temporarily
* hardcode a loop that increments PWM output to the wheels in increments of 100 and measure 
* corresponding RPM values.) Plot these measurements in Microsoft Excel (or equivalent) and obtain
* the line of best fit. Finally, manipulate this equation to obtain the PWM output for an RPM input.
*
* Inputs:   float RPM - The desired RPM of the wheel
*
* Output:   int PWM - The PWM output to obtain the desired RPM
***************************************************************************************************/
int MapRPMToPWM(float RPM)
{
    int PWM;
    if(RPM > 0) 
    {
        PWM = round((RPM + 1.0776808965)/0.0287579686); // The rearranged line of best fit equation
        PWM = clamp(PWM, MAX_PWM, 0);   // Clamp PWM to valid value
    }
    else PWM = 0;
    return PWM;
}

/***************************************************************************************************
* SET WHEEL PWMS FUNCTION
*
* This function hardsets the PWMs for each wheel.
*
* Inputs:   int pwm0 - Backleft wheel PWM
            int pwm1 - Frontleft wheel PWM
            int pwm2 - Frontright wheel PWM
            int pwm3 - Backright wheel PWM
***************************************************************************************************/
void Set_Wheel_PWMs(int pwm0, int pwm1, int pwm2, int pwm3)
{
    drive_pwm[0] = pwm0;
    drive_pwm[1] = pwm1;
    drive_pwm[2] = pwm2;
    drive_pwm[3] = pwm3;
}

/***************************************************************************************************
* SET DESIRED WHEEL SPEEDS FUNCTION
*
* This function hardsets the desired RPMs for each wheel.
*
* Inputs:   int rpm0 - Backleft wheel RPM
            int rpm1 - Frontleft wheel RPM
            int rpm2 - Frontright wheel RPM
            int rpm3 - Backright wheel RPM
***************************************************************************************************/
void Set_Desired_Speeds(int rpm0, int rpm1, int rpm2, int rpm3)
{
    desired_RPM[0] = rpm0;
    desired_RPM[1] = rpm1;
    desired_RPM[2] = rpm2;
    desired_RPM[3] = rpm3;
}

/***************************************************************************************************
* SET WHEEL DIRECTIONS FUNCTION
*
* This function takes in a direction for the rover to drive in (state machine: int input corresponds 
* to a different state) and applies the appropriate steering modifications.
*
* Inputs:   int direction - The desired driving direction:
                                - 0 (NORMAL DRIVE): All wheels spin in the same direction
                                - 1 (ON THE SPOT LEFT): Left wheels spin backwards, right forwards
                                - 2 (ON THE SPOT RIGHT): Right wheels spin forwards, left backwards
***************************************************************************************************/
void Set_Wheel_Directions(int direction)
{
    switch(direction)
    {
        case NORMAL_DRIVE:
            steer_mod[0] = 1;
            steer_mod[1] = 0;
            steer_mod[2] = 0;
            steer_mod[3] = 1;
            break;
        case ON_THE_SPOT_LEFT:
            steer_mod[0] = 1;
            steer_mod[1] = 1;
            steer_mod[2] = 1;
            steer_mod[3] = 1;
            break;
        case ON_THE_SPOT_RIGHT:
            steer_mod[0] = 0;
            steer_mod[1] = 0;
            steer_mod[2] = 0;
            steer_mod[3] = 0;
            break;
        default:
            // NORMAL DRIVE by default
            steer_mod[0] = 1;
            steer_mod[1] = 0;
            steer_mod[2] = 0;
            steer_mod[3] = 1;
    }

}

/********************************************************************
* COMMAND DATA CALLBACK FUNCTION
*
* The callback function for the subscription to drive command topic $
* called whenever new "cmd_data" data is published and sets the new $
* steering percentage for the rover via global variables.
*
* Input:    const rover::DriveCmd::ConstPtr& msg) - The message obje$
********************************************************************/
void arm_cmd_data_cb(const rover::ArmCmd::ConstPtr& msg)
{
  ROS_INFO("rcvd arm cmd");

  arm_fd = serialOpen("/dev/ttyACM0", 57600); 
  
  serialPrintf(arm_fd, "%d %d %d %d %d %d %d %d\n", -msg->base,
    msg->shoulder, -msg->forearm, -msg->wrist_x, -msg->wrist_y,
    -msg->twist, -msg->end_angle, -msg->end_pos);

  serialClose(arm_fd);
}


/***************************************************************************************************
* COMMAND DATA CALLBACK FUNCTION
*
* The callback function for the subscription to drive command topic "cmd_data". This function is 
* called whenever new "cmd_data" data is published and sets the new acceleration and 
* steering percentage for the rover via global variables.
*
* Input:    const rover::DriveCmd::ConstPtr& msg) - The message object containing the relevent data
***************************************************************************************************/
void cmd_data_cb(const rover::DriveCmd::ConstPtr& msg)
{   
    int speedL, speedR;
    // Get driving and steering commands from the mainframe
    if(!ball_tracking_enabled)
    {
        drive_pcnt = msg->acc;
        steer_pcnt = msg->steer;

        // Check to see if we are turning on the spot
        if(fabs(drive_pcnt) < ON_THE_SPOT_THRESHOLD)
        {
            if(steer_pcnt < 0) Set_Wheel_Directions(ON_THE_SPOT_LEFT);
            else Set_Wheel_Directions(ON_THE_SPOT_RIGHT);

            speedL = limit_drive*fabs((steer_pcnt*MAX_RPM)/100); 
            speedR = speedL;     
        }
        // Otherwise drive normally
        else
        {
            Set_Wheel_Directions(NORMAL_DRIVE);
            // Applies normal steering if desired. Steering is done by slowing down one side of wheels and speeding
            // up the other.
            if(steer_pcnt > 0)
            {
                speedL = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)-(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100)); 
                speedR = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)+(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100));
		    }
            else
            {
                speedL = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)+(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100)); 
                speedR = limit_drive*fabs((drive_pcnt*MAX_RPM)/100)-(limit_steer*fabs(steer_pcnt*(MAX_RPM/2)/100));
            }
        }
        Set_Desired_Speeds(speedR, speedR, speedL, speedL); // Set new desired wheel speeds
    }
      
}

/***************************************************************************************************
* ENCODERS CALLBACK FUNCTION
*
* The callback function for the subscription to the encoder data. This function is 
* called whenever new RPM calculations are sent by the Arduino and saves the RPM values into the
* global array actual_RPM so the data may be used in the motor PID controller.
*
* Input:    const rover::DriveCmd::ConstPtr& msg - The message object containing the relevent data;
            four fields of ints contain the RPM values for each wheel.
***************************************************************************************************/
void encoders_cb(const rover::RPM::ConstPtr& msg)
{   
    // Record data into array index respective to each wheel
    actual_RPM[0] = msg->rpm_bl;
    actual_RPM[1] = msg->rpm_fl;
    actual_RPM[2] = msg->rpm_fr;
    actual_RPM[3] = msg->rpm_br;
}

/***************************************************************************************************
* LIMIT DRIVE CALLBACK FUNCTION
*
* The callback function for the subscription to new drive limit value. This function sets the drive
* limit on the rover to the value included in the message.
*
* Input:    const std_msgs::Float32::ConstPtr& msg - The message data should be a single float 
            between 0-1, representing the maximum drive power between 0-100%.
***************************************************************************************************/
void limit_drive_cb(const std_msgs::Float32::ConstPtr& msg)
{
    limit_drive = fclamp(msg->data,1.0,0.0); // Clamp value to within 0-100% limit
    ROS_INFO_STREAM("ROVER: SETTING SPEED LIMIT TO " << (limit_drive*100) << "%");
}

void ball_cb(const rover::Ball::ConstPtr& msg)
{
    if(ball_tracking_enabled)
    {
        ball_x = msg->x;
        ball_radius = msg->radius;
        if(ball_x < ((BALL_CAMERA_WIDTH/2) - BALL_TRACK_MIDDLE_WIDTH))
        {
            ROS_INFO_STREAM("BALL TURN LEFT");
            Set_Wheel_Directions(ON_THE_SPOT_LEFT);

            Set_Desired_Speeds(BALL_TRACK_SPEED, BALL_TRACK_SPEED, BALL_TRACK_SPEED, BALL_TRACK_SPEED);
        }
        else if((ball_x > ((BALL_CAMERA_WIDTH/2) + BALL_TRACK_MIDDLE_WIDTH)) || (ball_x == -1))
        {
            ROS_INFO_STREAM("BALL TURN RIGHT OR BALL UNLOCATED");
            Set_Wheel_Directions(ON_THE_SPOT_RIGHT);
            Set_Desired_Speeds(BALL_TRACK_SPEED, BALL_TRACK_SPEED, BALL_TRACK_SPEED, BALL_TRACK_SPEED);
        }
        else 
        {
            if(ball_radius < BALL_MAX_RADIUS)
            {
                ROS_INFO_STREAM("BALL FORWARD");
                Set_Wheel_Directions(NORMAL_DRIVE);

                Set_Desired_Speeds(BALL_TRACK_SPEED, BALL_TRACK_SPEED, BALL_TRACK_SPEED, BALL_TRACK_SPEED);
            }
            else 
            {
                ROS_INFO_STREAM("ROVER: BALL LOCATED, SETTING RPM TO 0");

                Set_Wheel_PWMs(0, 0, 0, 0);
                Set_Desired_Speeds(0, 0, 0, 0);
                ball_tracking_enabled = 0;

		drive_pcnt    = 0;  
		steer_pcnt    = 0;  

                n->setParam("/STATE", "DRIVE"); // Exit autonomous mode when done                
            }
        }
    }
}

/***************************************************************************************************
* SIGINTHANDLER FUNCTION
*
* This function overrides the default ROS sigint handler for Ctrl+C.
*
* Input:    int sig - The signal(? - Ask Ben)
***************************************************************************************************/
void SigintHandler(int sig)
{
    // Disable power to the relay
    digitalWrite (RELAY_1_PIN, 0);
    digitalWrite (RELAY_2_PIN, 0);
    
    serialClose(arm_fd);

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    // ********************* ROS ************************* //

    ros::init(argc, argv, "underling");
    n = new ros::NodeHandle();
    ros::Rate loop_rate(LOOP_HERTZ);
  
    // Declare publishers and subscribers
    ros::Subscriber     drivecmd_sub    = n->subscribe("cmd_data", 5, cmd_data_cb);
    ros::Subscriber     armcmd_sub      = n->subscribe("arm_cmd_data", 1, arm_cmd_data_cb);
    ros::Subscriber     encoders_sub    = n->subscribe("/encoders", 5, encoders_cb);		
    ros::Subscriber     limit_drive_sub = n->subscribe("/limit_drive", 1, limit_drive_cb);
    ros::Subscriber     ball_sub        = n->subscribe("/ball", 1, ball_cb);
    ros::Publisher      reqRPM_pub      = n->advertise<rover::ReqRPM>("req_rpm", 4);

    string state; // Holds the current parameter state
    bool drive_dir = 1; // Wheel direction
    bool standby_message_printed = 0;
  
    signal(SIGINT, SigintHandler); // Override the default ros sigint handler.
    // Enable power to the RELAYs
    digitalWrite (RELAY_1_PIN, 1);
    digitalWrite (RELAY_2_PIN, 1);

    // ****************** VARIABLES ********************** //

    const int dir_pins[4] = {F_L_DIR_PIN, 
                           F_R_DIR_PIN, 
                           B_L_DIR_PIN, 
                           B_R_DIR_PIN};  

    // PID variables
    float error[4]          = {0,0,0,0};
    float error_prior[4]    = {0,0,0,0};
    float integral[4]       = {0,0,0,0};
    float derivative[4]     = {0,0,0,0};
    float output[4]         = {0,0,0,0};

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

    // Enable power to the RELAYs
    pinMode(RELAY_1_PIN, OUTPUT);
    pinMode(RELAY_2_PIN, OUTPUT);
    digitalWrite (RELAY_1_PIN, 0);
    digitalWrite (RELAY_2_PIN, 0);

    pinMode(ZERO_RST_PIN, OUTPUT);
    digitalWrite (ZERO_RST_PIN, 1); // Reset low

    ros::Duration(3).sleep(); // Allow some time for setup

    n->setParam("/MODE", "TEST"); // Make sure we start in testing mode

    // *************************** MAIN LOOP *************************** //

    while (ros::ok())
    {
        // Set new motor directions
        drive_dir = !(drive_pcnt < 0);

        // Speed control loop; loops for each wheel
        for(int k = 0; k < 4; k++) 
        {
            error[k]        = desired_RPM[k] - actual_RPM[k];
            integral[k]     = fclamp((integral[k] + (error[k] * iteration_time)), 25.0, -25.0);
            derivative[k]   = (error[k] - error_prior[k]) / iteration_time;
            output[k]       = (K_P*error[k]) + (K_I*integral[k]) + (K_D*derivative[k]);
            error_prior[k] 	= error[k];

            drive_pwm[k]    = MapRPMToPWM(round(actual_RPM[k]+output[k]));
        }

        // Reset Arduino when switching to ARM mode
        n->getParam("/STATE", state);
        if ((state == "ARM") && (prev_state != state))
        {
          ROS_INFO("resetting arduino");
          digitalWrite (ZERO_RST_PIN, 0); // Reset high
          ros::Duration(0.1).sleep();
          digitalWrite (ZERO_RST_PIN, 1); // Reset low

          ros::Duration(1.0).sleep(); // Pause to let Arduino start up
        }
        else if ((prev_state == "ARM") && (state != "ARM"))
        {
          arm_fd = serialOpen("/dev/ttyACM0", 57600);  
          serialPrintf(arm_fd, "0 0 0 0 0 0 0 0\n");
          serialClose(arm_fd);
        }
        prev_state = state;
  

        if(state == "STANDBY")
        {
            digitalWrite (RELAY_1_PIN, 0);
            digitalWrite (RELAY_2_PIN, 0);
            Set_Wheel_PWMs(0, 0, 0, 0);
            Set_Desired_Speeds(0, 0, 0, 0);
            ball_tracking_enabled = 0;

            if(!standby_message_printed)
            {
                ROS_ERROR_STREAM("ROVER: STATE SET TO STANDBY - SETTING PWM TO 0");
                standby_message_printed = 1;
            }
        }
        else
        {
            n->getParam("/AUTO_STATE", state);
            if(state == "SEARCH")
            {
                ball_tracking_enabled = 1;
            }
            else
            {
                digitalWrite (RELAY_1_PIN, 1);
                digitalWrite (RELAY_2_PIN, 1);
                standby_message_printed = 0;
                ball_tracking_enabled = 0;
            }
        }

        // Update wheel directions and speeds
        for (int i = 0; i < 4; i++)
        {
            bool direction;
            direction = (steer_mod[i] != drive_dir);
            digitalWrite (dir_pins[i], direction);

            pwmWrite(PIN_BASE + i, drive_pwm[i]);
        }

        // Publish desired wheel speeds for sanity checker
        // todo: change mainframe code to publish just desired RPM values rather than steer/drive %
        rover::ReqRPM msg;
        msg.req_rpm_fl = desired_RPM[0];
        msg.req_rpm_fr = desired_RPM[1];
        msg.req_rpm_bl = desired_RPM[2];
        msg.req_rpm_br = desired_RPM[3];
        reqRPM_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
