# Message format for sending drill commands to rover.

bool kill #FailSafe 1 if Kill, 0 if Ok. X Button for Kill Switch

int16 drillSpd # -255 TO +255 PWM// Constant value for the moment. Controlled by right and left Trigger
int16 stepperPos # -ve and +ve Positions. D Pad Left and Right
int16 actuatorSpeed # -255 TO +255 PWM, 300Val is Cache Position Height. Left Joystick Up Down
int16 sensorSpeed # -255 TO +255 PWM Right Joystick Up Down
