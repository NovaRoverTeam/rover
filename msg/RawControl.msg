# Compact message for user controller input.
#
# Type:
#       0: Button
#       1: Stick
#       2: Trigger

int8 type     
int8 id         # Unique identification number
bool depress    # True if button depressed, false if released
float64 length  # Normalised length of stick or trigger value

# Added these on 07/05/17 *****************************
float64 axis_x  # Normalised stick x
float64 axis_y  # Normalised stick y
