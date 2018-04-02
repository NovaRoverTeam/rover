# Message format for sending drill commands to rover.

bool kill #FailSafe 1 if Kill, 0 if Ok

int8 drillSpd # -255 TO +255 PWM// Constant value for the moment
int8 stepperPos # -ve and +ve Positions
int8 actuatorSpeed # -255 TO +255 PWM, 300Val is Cache Position Height
