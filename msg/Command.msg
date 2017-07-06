# Message format for sending commands to rover.
#   TODO: Implement enumerators for values
#
# System:
#   0: Drive system
#   1: Manipulator
#   2: Camera
#
# Target:
#   Drive System:
#     0: Wheel motors  
#
# Command:
#   Drive System:
#     0: Stop
#     1: Drive
#     2: Steer

int8 system     
int8 target  # Target subsystem
int8 command # Command to be sent

float64 value1 # left wheels
float64 value2 # right wheels
