# Message format for sending drill commands to rover.

bool kill #FailSafe 1 if Kill, 0 if Ok

int16 drillSpd # -255 TO +255 PWM// Constant value for the moment
int16 stepperPos # -ve and +ve Positions
int16 actuatorSpeed # -255 TO +255 PWM, 300Val is Cache Position Height
