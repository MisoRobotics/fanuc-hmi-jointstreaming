# fanuc-hmi-jointstreaming
Alpha implementation of joint position streaming via HMI

## Setup
Requirements:
Fanuc options:
- R553 HMI Device (SNPX)
System variables:
- $SNPX_PARAM.$NUM_MODBUS = 2
- $SNPX_ASG[76]:
  -	$ADDRESS = 16300
  -	$SIZE = 2
  -	$VAR_NAME = $SNPX_ASG[80].$ADDRESS
  -	$MULTIPLY = 1.000
- $SNPX_ASG[77]:
  -	$ADDRESS = 16302
  -	$SIZE = 2
  -	$VAR_NAME = $SNPX_ASG[80].$SIZE
  -	$MULTIPLY = 1.000
- $SNPX_ASG[78]:
  -	$ADDRESS = 16304
  -	$SIZE = 40
  -	$VAR_NAME = $SNPX_ASG[80].$VAR_NAME
  -	$MULTIPLY = -1.000
- $SNPX_ASG[79]:
  -	$ADDRESS = 16344
  -	$SIZE = 2
  -	$VAR_NAME = $SNPX_ASG[80].$MULTIPLY
  -	$MULTIPLY = 0.000
IO:
- DO[3] should be mapped to UI[6]
- DO[4] should be mapped to UI[2]
- DO[5] should be mapped to UI[5]
- DO[6] should be mapped to UI[4]

I implemented these mappings via a MemPort (rack 0, slot 0, start 1)
- DI[118-120] and DI[185] should be mapped to safety scanner inputs so the system can monitor when robot is going to be halted and stop streaming.


## Demo
Launch the system using `roslaunch miso_fanuc fanuc-drivers.launch robot_ip:=<IP address>`

After the system launches, there is a demo script that demonstrates position streaming.
The script first moves a 6-axis robot to [0, 0, 0, 0, 0, 0], and then moves J1 in a sine wave pattern for 10 seconds.
Run the motion demo script using `rosrun miso_fanuc motion_demo`

## Notes
This is an alpha implementation of joint position streaming using HMI, thus it is lacking in some features that would be needed
in a production environment. For example, the current streaming implementation does not implement any feedback from joint positions, it just steps through the trajectory
in time and interpolates points to send as the next setpoint. The time-scaling factor can be reduced from 1 if something is
causing the robot to run slower than the trajectory (E-stop event, operator speed override), but if the robot momentarily stops for whatever reason while the time
scaling factor remains at 1, there could be jumps in motion, potentially through in an obstacle in environment that the trajectory would have navigated around.