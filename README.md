MiniArmDriver is the Arduino sketch that simply receives servo position commands over the serial port from the Processing sketch MiniRobotArm.

The format is servo_#,servo_position
where servo_# is a number from 0 to 5
servo_position is the position of the servo in degrees.

Example:
0,90  // move servo 0 to 90 degrees

Note that the default baud rate is 38400

MiniRobotArm is the processing sketch that sends commands to MiniArmDriver.

Required Processing Libraries (can be installed directly in Processing):
Leap Motion for Processing by Darius Morawiec
G4P by Peter Lager

If you want to edit the user interface install:
G4PTool by Peter Lager directly from the Processing interface

