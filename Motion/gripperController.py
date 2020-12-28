from __future__ import print_function
from __future__ import division
import roslib
import rospy
import time
import threading
import sys
from copy import copy,deepcopy

from std_srvs.srv import Empty

from reflex_msgs2.msg import Command
from reflex_msgs2.msg import PoseCommand
from reflex_msgs2.msg import VelocityCommand
from reflex_msgs2.msg import Hand
from reflex_msgs2.msg import Motor
from reflex_msgs2.msg import FingerPressure
from reflex_msgs2.srv import SetTactileThreshold, SetTactileThresholdRequest


import sys

# Constants here
MAX_MOTOR_SPEED = 3.5
MIN_MOTOR_SPEED = -3.5
MAX_MOTOR_TRAVEL = 3.6
MIN_MOTOR_TRAVEL = 0


hand_state = Hand()

class GripperController:

#############################################
###These functions are not for user to use###
#############################################

    def __init__(self):
        self.command_pub = rospy.Publisher('/reflex_takktile2/command', Command, queue_size=5)
        self.pos_pub = rospy.Publisher('/reflex_takktile2/command_position', PoseCommand, queue_size=5)
        self.vel_pub = rospy.Publisher('/reflex_takktile2/command_velocity', VelocityCommand, queue_size=5)
        self.sub = rospy.Subscriber('/reflex_takktile2/hand_state', Hand, self._callback)

        self.dt = 1.0/30.0
        self.ds = 0.3*self.dt
        self.pressure_threshold = 20
        self.torque_threshold = 350
        self.soft_torque_threshold = 150
        self.sensed_finger_set = [0.0, 0.0, 0.0, 0.0]
        self.command_finger_set = [0.0, 0.0, 0.0, 0.0]

        self.command_type = None
        self.guarded_move = False
        self.contact = False
        self.finger_contact = [False, False, False]
        self.motor_load = [0.0, 0.0, 0.0, 0.0]
        self.finger_pressure_set = [None, None, None]
        self.enable = False
        self.exit = False
        self.paused = False
        self.new_state = False
        self.flag = [False, False, False] # may need to reset every time
        self.logs = []




    def start(self):
        self.enable = True
        # rospy.init_node('gripperController')
        controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()


    def _controlLoop(self):
        now = rospy.get_rostime()

        self.lastStateTime = float(now.secs) + 1e-9*float(now.nsecs) #get the ros time for each robot state msg
        rate = rospy.Rate(1.0/self.dt)

        while not rospy.is_shutdown() and (not self.exit):
            # f = open("gripperlog.txt", "a")
            # f.write("this is the finger position: ")
            # for x in self.sensed_finger_set:
            #     f.write(str(x))
            #     f.write("  ")
            # f.write("\n")
            # f.write("this is the motor load: ")
            # for x in self.motor_load:
            #     f.write(str(x))
            #     f.write("  ")
            # f.write("\n")
            # f.write("this is the finger pressure: ")
            # for x in self.motor_load:
            #     f.write(str(x))
            #     f.write("  ")
            # f.write("\n")
            # f.close()
            if self.paused:
                self.pos_pub.publish(PoseCommand(f1 = self.sensed_finger_set[0], f2 = self.sensed_finger_set[1], f3 = self.sensed_finger_set[2], preshape = self.sensed_finger_set[3]))
            else:
                if self.guarded_move:
                    for i in range(3):
                        if self.flag[i] == False:
                            for sensor in self.finger_pressure_set[i]:
                                if  self.motor_load[i] > self.soft_torque_threshold or sensor > self.pressure_threshold or sensor < -self.pressure_threshold or self.command_finger_set[i] > 3.0:
                                    print("finger stopped:",i, sensor, self.command_finger_set[i], self.motor_load[i])
                                    self.flag[i] = True
                                    self.command_finger_set[i] = self.command_finger_set[i] - 10.0*self.ds
                                    break

                        if self.flag[i] == False:
                            self.command_finger_set[i] = self.command_finger_set[i] + self.ds

                    self.pos_pub.publish(PoseCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))

                    for i in range(3):
                        if self.motor_load[i] > self.torque_threshold:
                             self.guarded_move = False
                             print("Guarded move exited because of torque")
                             break

                    if self.flag[0] == True and self.flag[1] == True and self.flag[2] == True:
                        self.guarded_move = False
                        print("Guarded move exited because of pressure")

                else:
                    if self.command_type == "pose":
                        self.pos_pub.publish(PoseCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))
                    elif self.command_type == "velocity":
                        self.vel_pub.publish(VelocityCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))


            rate.sleep()


    def _callback(self, data):
        # self.lastStateTime = float(data.header.stamp.secs)+1e-9*float(data.header.stamp.nsecs)
        self.sensed_finger_set[0] = data.motor[0].joint_angle
        self.sensed_finger_set[1] = data.motor[1].joint_angle
        self.sensed_finger_set[2] = data.motor[2].joint_angle
        self.sensed_finger_set[3] = data.motor[3].joint_angle
        self.finger_pressure_set[0] = data.finger[0].pressure
        self.finger_pressure_set[1] = data.finger[1].pressure
        self.finger_pressure_set[2] = data.finger[2].pressure
        self.logs.append((deepcopy(self.sensed_finger_set), deepcopy(self.finger_pressure_set)))
        self.contact = False
        self.finger_contact = [False, False, False]
        for i in range(3):
            for sensor in data.finger[i].contact:
                self.finger_contact[i] |= sensor
                if self.finger_contact[i] == True:
                    self.contact = True
                    break
        # for finger in data.finger:
        #     for sensor in finger.contact:
        #         self.contact |= sensor
        #         if self.contact == True:
        #             break
        #     if self.contact == True:
        #         break
        self.motor_load = [data.motor[0].load, data.motor[1].load, data.motor[2].load, data.motor[3].load]
        self.new_state = True



    def _regPose(self, position):
        for i in range(len(position)):
            if position[i] > MAX_MOTOR_TRAVEL:
                print("command position for gripper is too far, set to max position")
                position[i] = MAX_MOTOR_TRAVEL
            if position[i] < MIN_MOTOR_TRAVEL:
                print("command position for gripper is too far, set to max position")
                position[i] = MIN_MOTOR_TRAVEL


    def _regVelocity(self, velocity):
        for i in range(len(velocity)):
            if velocity[i] > MAX_MOTOR_SPEED:
                print("command velocity for gripper is too fast, set to max velocity")
                velocity[i] = MAX_MOTOR_SPEED
            if velocity[i] < MIN_MOTOR_SPEED:
                print("command velocity for gripper is too fast, set to max velocity")
                velocity[i] = MIN_MOTOR_SPEED



    ###funcitons for open/close###
    def setPose(self, position):
        """
        commands the fingers go to the position

        Parameters:
        Position:  an iteratable object of length 4, floats
        the first and second index denote the two fingers on the same side,
        the third index denote the one finger which is on the otherside,
        the fourth index denote the angle between the first and second fingers.
        all value in the array shoul be in range 0~3.6
        if not, the function will convert the input into this range


        """
        if not self.paused:
            self._regPose(position)
            self.command_finger_set[0] = position[0]
            self.command_finger_set[1] = position[1]
            self.command_finger_set[2] = position[2]
            self.command_finger_set[3] = position[3]
            self.command_type = "pose"


    def setVelocity(self, velocity):
        """
        commands the fingers go inward or outward
        if the velocity is positive, then the fingers will close
        if the velocity is negative ,then the fingers will open

        Parameters:
        velocity:  an iteratable object of length 4, floats
        the first and second index denote the two fingers on the same side,
        the third index denote the one finger which is on the otherside,
        the fourth index denote the angle between the first and second fingers.
        all value in the array shoul be in range -3.5~3.5
        if not, the function will convert the input into this range


        """
        if not self.paused:
            self._regVelocity(velocity)
            self.command_finger_set[0] = velocity[0]
            self.command_finger_set[0] = velocity[1]
            self.command_finger_set[0] = velocity[2]
            self.command_finger_set[0] = velocity[3]
            self.command_type = "velocity"


    def guarded_close(self):
        """
        it close the finger until the pressure is too high
        """
        if not self.paused:
            self.guarded_move = True
            self.command_finger_set = deepcopy(self.sensed_finger_set)
            self.command_type = "pose"

    def open(self):
        """
        this opens the finger to [0,0,0,0] Position
        """
        if not self.paused:
            self.command_finger_set[0] = 0.0
            self.command_finger_set[1] = 0.0
            self.command_finger_set[2] = 0.0
            self.command_finger_set[3] = 0.0
            self.command_type = "pose"



    #Calibrate will calibrate the finger and the sensor in the same time.
    #Calibration may move the fingers.
    #Also, calibrate sensor requires no contact

    def calibrate(self):
        """
        Calibrate will calibrate the finger and the sensor in the same time.
        Calibration may move the fingers.
        Also, calibrate sensor requires no contact
        Hence calibration need to be done manually
        """

        rospy.wait_for_service('/reflex_takktile2/calibrate_fingers')
        calibrate_fingers = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
        rospy.wait_for_service('/reflex_takktile2/calibrate_tactile')
        calibrate_tactile = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
        calibrate_fingers()
        time.sleep(5)
        calibrate_tactile()
        time.sleep(1)

####################################
###  probably won't be used now  ###
####################################
    def set_threshold(self, x):
        set_tactile_threshold = rospy.ServiceProxy('/your_gripper/set_tactile_threshold', SetTactileThreshold)
        set_tactile_threshold(x)



##############################################
###these functions return the current state###
##############################################

    def is_open(self):
        """
        this function returns if the gripper is open

        Returns:
        if the gripper is open return True
        otherwise, return False
        """
        if self.sensed_finger_set[0] - 0 < 0.1 and self.sensed_finger_set[1] - 0 < 0.1 and self.sensed_finger_set[2] - 0 < 0.1:
            return True
        else:
            return False

    def contact_with_object(self):
        """
        this function returns if any one of the fingers is contacted with the object

        Returns:
        if the gripper is contacted with object return True
        otherwise, return False
        """
        return self.contact

    def sensed_finger_positions(self, index):
        """
        This function gives the current position of one finger

        Parameters:
        index: the index of the finger

        Returns:
        the position of the finger
        """
        return self.sensed_finger_set[index]

    def finger_contact_with_object(self, index):
        """
        This function returns if one specific finger is contact with the object

        Parameters:
        index the index of the finger

        Returns:
        if the finger is contacted with object return True
        otherwise return False
        """

        return self.finger_contact[index]

    def finger_pressure(self, index):
        """
        This function gives the current pressure of one finger

        Parameters:
        index: the index of the finger

        Returns:
        the pressure of the finger
        """
        return self.finger_pressure_set[index]

    def motorload(self):
        """
        This function returns the motorload of the 4 motor

        Returns:
        A 4-length array which first is the motor for finger1
        second is the motor for finger2
        third is the motor for finger3
        fourth is the motor for the preshape
        """
        return self.motor_load



    def pause(self):
        """
        This function stops the stop_process
        """

        self.enable = False
        self.paused = True
        self.guarded_move = False
        #set the command to go to current position. Note that this won't get executed (main control loop does something else)
        #this will act to clear other type of commands 

        self.setPose(copy(self.sensed_finger_set))


    def resume(self):
        """
        This function resume the process
        """
        self.enable = True
        self.paused = False

    def shutDown(self):
        """
        This function shutdown the stop_process
        """
        self.enable = False
        self.exit = True
        time.sleep(0.1)


    def moving(self):
        if self.enable:
            return True
        else:
            return False

    def mark_read(self):
        self.new_state = False

    def newState(self):
        return self.new_state

    def is_guarded_moving(self):
        return self.guarded_move


if __name__ == "__main__":
    con = GripperController()
    con.start()
    # con.fopen()
    # con.calibrate()
    # con.guarded_close()
    con.open()
    # con.setPose([0, 0.5, 0, 0])
    time.sleep(2)
    # startTime = time.time()
    while(con.is_guarded_moving()):
        # print("finger1 pressure: {0}\nfinger2 pressure: {1}\nfinger3 pressure: {2}\n".format(con.finger_pressure(0), con.finger_pressure(1), con.finger_pressure(2)))
        # print("finger1 position: {0}\nfinger2 position: {1}\nfinger3 position: {2}\n preshape: {3}\n".format(con.sensed_finger_positions(0), con.sensed_finger_positions(1), con.sensed_finger_positions(2), con.sensed_finger_positions(3)))
        print("motor load is: ", con.motorload())
    #
        # print("finger1 is contacted: {0}\nfinger2 is contacted: {1}\nfinger3 is contacted: {2}\nfinger is contacted: {3}\n".format(con.finger_contact_with_object(0), con.finger_contact_with_object(1),con.finger_contact_with_object(2), con.contact_with_object()))
        time.sleep(0.05)
    #     if con.contact_with_object():
    #         break
    # time.sleep(5)
    # # con.open()
    # # # con.close()
    # time.sleep(1)
    con.shutDown()
    # print(con.sensed_finger_positions())
    # print(con.motorload())
