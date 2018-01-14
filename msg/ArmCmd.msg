# Message format for sending manipulator commands to rover.

int8 base     # -1 left, 0 none/both or 1 right
int8 shoulder # -1 down, 0 none/both or 1 up
int8 forearm  # etc
int8 wrist_x
int8 wrist_y
int8 twist
int8 grip

int8 sensitivity  # movement speed, 1-5
