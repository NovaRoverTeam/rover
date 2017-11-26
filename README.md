# rover
Contains the main control program to run on the Mars rover.

## Dependencies:

- WiringPi (http://wiringpi.com/download-and-install/)

- pca9685 (https://github.com/Reinbert/pca9685)

## Subscriptions:

Topic:       hbeat
Msg type:    std_msgs/Empty
Description: Listens for heartbeat from base station in order to determine whether or not we have lost comms.

Topic:       cmd_data
Msg type:    rover/DriveCmd (custom)
Description: Receives speed and steering commands from the base station.

Topic:       encoders
Msg type:    rover/Encoders (custom)
Description: Receives RPM values reported by the Arduino connected to the wheel encoders.
