/********************************************************************

  HARRY POTTER DRIVE CODE

  This script controls the driving code for the four wheeled rover.
  It takes in two inputs (a forward speed and a directional speed) and
    outputs eight values (directional values and PWM values for each of
    the four wheels).

********************************************************************/

/********************************************************************
  VARIABLES AND DEPENDENCIES
********************************************************************/

// Include the dependencies
#include <iostream>
#include <cmath>
#include <string>
#include <stdlib.h>
using namespace std;

#define N_WHEELS 4 // Number of wheels

// Input values ****MOVE TO ROS SERVICE***
float input_forward = 0; // Stores the forward input value (-100, 100)
float input_directional = 0; // Stores the directional input value (-100, 100)

// Stores the maximum PWM value
const float PWM_Max = 4096;

// PWM values for the wheels
int drive_pwm[] = {0, 0, 0, 0};

// Directional values for the wheels
// A value of 0 is clockwise, a value of 1 is counterclockwise
int drive_dir[] = {0, 0, 1, 1};

// Lists the description of each wheel
string drive_desc[] = {"Front Left", "Back Left", "Front Right", "Back Right"};

// The order of these values goes:
//  0: Front LEFT
//  1: Back LEFT
//  2: Front RIGHT
//  3: Back RIGHT




/********************************************************************
  MATHEMATICAL OPERATIONS AND FUNCTIONS
********************************************************************/

// Takes in an input and output range of values and maps one set to the other
float Map (float in_min, float in_max, float out_min, float out_max, float value) {
  float perc = (value - in_min) / (in_max - in_min);
  float output = perc * (out_max - out_min) + out_min;
  return output;
}

// Clamps a float between a minimum and maximum
float Clamp (float min, float max, float value) {
  if (value < min)
    return min;
  else if (value > max)
    return max;
  else
    return value;
}


/********************************************************************
  WHEEL SPEED CALCULATIONS
********************************************************************/

// Flips all the wheel directions
void Flip_Wheels () {
  for (int i = 0; i < N_WHEELS; i++) {
    drive_dir[i] = abs(drive_dir[i] - 1);
  }
}

// Calculates the set of all PWM Values
void Calculate_PWM_Values (float forward, float directional) {

  // Store temporary float PWMs
  float temp_pwm[] = {0, 0, 0, 0};

  // Scale the forward factor depending on the Max value
  float for_factor = Map(0, 100, 0, PWM_Max, abs(forward));

  // Sets a starting PWM factor depending on the forward scalar
  for (int i = 0; i < N_WHEELS; i++) {
    temp_pwm[i] = (i < 2) ? for_factor : -for_factor;
  }

  // Map a directional value depending on how much the rover is turning
  // If turning right
  if (directional > 0) {
    float dir_factor = 1.0 - (2 * Map(0, 100, 0, 1.0, directional));
    temp_pwm[2] *= dir_factor;
    temp_pwm[3] *= dir_factor;
  } else { // If turning left
    float dir_factor = (2 * Map(-100, 0, 0, 1.0, directional)) - 1.0;
    temp_pwm[0] *= dir_factor;
    temp_pwm[1] *= dir_factor;
  }

  // Flip wheel directions if PWM < 0
  for (int i = 0; i < N_WHEELS; i++) {
    if (temp_pwm[i] < 0) {
      drive_dir[i] = 1;
    } else {
      drive_dir[i] = 0;
    }

    // Ensure the PWM values are absolute between 0 and 1
    temp_pwm[i] = abs(temp_pwm[i]);

    // Convert the float PWM to a integer one
    drive_pwm[i] = (int)temp_pwm[i];
  }

  // If going backwards, flip direction
  if (forward < 0) {
    Flip_Wheels();
  }
}




/********************************************************************
  MAIN FUNCTIONS
********************************************************************/

// Main function runs when program is executed
int main(int argc, char** argv) {

  // Get arguments from main function
  // THIS WILL BE REMOVED WITH ROS MESSAGES
  if (argc > 1) {
    input_forward = atof(argv[1]);

    // A directional input is optional
    if (argc > 2) {
      input_directional = atof(argv[2]);
    }
  }

  // Calculate the new PWM values
  Calculate_PWM_Values(input_forward, input_directional);

  // Output PWM and Directional Values to the console
  for (int i = 0; i < N_WHEELS; i++) {
    cout << drive_desc[i] << "\t" << drive_pwm[i] << ", \t" << drive_dir[i] << endl;
  }

  return 0;
}
