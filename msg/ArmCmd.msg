# Message format for sending manipulator commands to rover.

bool start # Start button down?
bool back  # Back button down?

int16 base     # -1 left, 0 none/both or 1 right
int16 shoulder # -1 down, 0 none/both or 1 up
int8 forearm  # etc
int16 wrist_x
int16 wrist_y
int8 twist
int8 end_angle
int8 end_pos

int8 sensitivity  # movement speed, 1-5
