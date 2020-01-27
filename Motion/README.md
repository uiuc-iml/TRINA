naming conventions:
functions: isRobotMoving()
objects: left_arm
classes: leftArm()

gripper:
First do
$roslaunch reflex reflex_takktile2.launch
if it shows something with network issue, try the following step:
"Network Connection" -> "wired connection 1" -> "edit" -> "IPv4 Setting"
Method: Manual
Addresses: Address: 10.1.1.10 Netmask: 254.0.0.0 Gateway: 0.0.0.0
check Require IPv4 addressing for this connection to complete
Save
Make sure shutdown is called. Otherwise the gripper may not work. You may need to close the terminal and then open a new one.


##
The arms need to be in static ip as well. With the same mask and gateway as the PC
