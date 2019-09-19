Install the PyUniversalRobot package first by running setupPy2.py
-- currently the communication frequency is set to be 250Hz. To change this, go to network/rtde.py
   and change the frequency 

To use the api, do
 --- from robot_api.RobotController import UR5WithGripperController

The source code is in RobotController.py and UR5_controller.py. 

RobotController.py imports UR5_controller.py. UR5_controller.py interfaces with the UR5 robot while RobotController.py provides
a unified controller for both the robot and gripper. 

The UR5WithGripperController class provides a series of functions to interface and control the robot and gripper. Example code:

    from robot_api.RobotController import UR5WithGripperController

    ur5 = UR5WithGripperController(host='10.10.1.104', gripper=False) #input includes IP address  & not using gripper
    ur5.start()  # start the robot


    ## control loop, running a sinusoidal motion 
    start_time=time.time()
    while time.time()-start_time < 15:
        t=time.time()-start_time
        q1=0.7*math.sin(t)
        q3=0.7*math.sin(t)
        position = [q1,-math.pi/2,q3,-math.pi/2,0,0,0]
        ur5.setConfig(position)
        time.sleep(0.005)

    ur5.stop()  # must call this function at the end to shut down controller properly



setConfig() takes a list of 7 elements, the first 6 are the robot joint positions and the last is the gripper. Set the last to
0 if gripper not being used.
You can also set the velocity of the joints and the gripper instead of positions.
Other functions include getConfig(), getVelocity(), getWrench(),getCurrentTime(). See the source code for more details.







