import roslib
import rospy
import time
import threading

from __future__ import print_function
from __future__ import division

from std_srvs.srv import Empty

from reflex_msgs2.msg import Command
from reflex_msgs2.msg import PoseCommand
from reflex_msgs2.msg import VelocityCommand
from reflex_msgs2.msg import Hand
from reflex_msgs2.msg import FingerPressure
from reflex_msgs2.srv import SetTactileThreshold, SetTactileThresholdRequest


import sys

# Constants here
MAX_MOTOR_SPEED = 3.5
MIN_MOTOR_SPEED = -3.5
MAX_MOTOR_TRAVEL = 3.6
MIN_MOTOR_TRAVEL = 0

class GripperController:
    def __init__(self):
        self.command_pub = rospy.Publisher('/reflex_takktile2/command', Command, queue_size=10)
        self.pos_pub = rospy.Publisher('/reflex_takktile2/command_position', PoseCommand, queue_size=10)
        self.vel_pub = rospy.Publisher('/reflex_takktile2/command_velocity', VelocityCommand, queue_size=10)
        self.sub = rospy.Subscriber('/reflex_takktile2/hand_state', Hand, self.callback)

        self.dt = 0.01

        self.sensed_finger_1 = 0.0
        self.sensed_finger_2 = 0.0
        self.sensed_finger_3 = 0.0
        self.sensed_preshape = 0.0
        self.commanded_finger_1 = 0.0
        self.sensed_finger_2 = 0.0
        self.sensed_finger_3 = 0.0
        self.sensed_preshape = 0.0

        self.method = NULL

        
        self.enable = False
        self.exit = False

    def start(self):
        self.enable = True
        rospy.init_node('gripperController')
        controlThread = threading.Thread(target = self.controlLoop)
        controlThread.start()

    def controlLoop(self):
        now = rospy.get_rostime()
        self.lastStateTime = float(now.secs) + 1e-9*float(now.nsecs) #get the ros time for each robot state msg
		rate = rospy.Rate(1.0/self.dt)

        self.calibrate()
        while not rospy.is_shutdown() and (not self.exit):

            if stop_flag:
                set velocity zero
            if self.method == "pose":
                self.pos_pub.publish(PoseCommand(f1 = self.fingerone, f2 = self.fingertwo, f3 = self.fingerthree, preshape = self.preshape))
                self.method = NULL
            elif self.method == "velocity":
                self.vel_pub.publish(VelocityCommand(f1 = self.fingerone, f2 = self.fingertwo, f3 = self.fingerthree, preshape = self.preshape))
                self.method = NULL

            rate.sleep()

    def moving(self):
        if not self.enable:
            return False
        else:
            return True
    def callback(self, data):
        self.lastStateTime = float(data.header.stamp.secs)+1e-9*float(data.header.stamp.nsecs)
        self.sensed_finger_1 = data.motor[0].joint_angle
        self.fingertwo = data.motor[1].joint_angle
        self.fingerthree = data.motor[2].joint_angle
        self.preshape = data.motor[3].joint_angle
        self.newStateFlag = True

    def setPose(self, position):
        now = rospy.get_rostime()
        currentTime = float(now.secs) + 1e-9*float(now.nsecs)
        regPose(position)
        self.fingerone = position[0]
        self.fingertwo = position[1]
        self.fingerthree = position[2]
        self.preshape = position[3]
        self.method = "pose"

    def regPose(position):
        for i in range(len(position)):
            if position[i] > MAX_MOTOR_TRAVEL:
                print("command position for gripper is too far, set to max position")
                position[i] = MAX_MOTOR_TRAVEL
            if position[i] < MIN_MOTOR_TRAVEL:
                print("command position for gripper is too far, set to max position")
                position[i] = MIN_MOTOR_TRAVEL

    def close(self):
        self.close = True
        self.fingerone = 3.5
        self.fingertwo = 3.5
        self.fingerthree = 3.5
        self.preshape = 0
        self.method = "pose"

    def open(self):
        self.fingerone = 0.0
        self.fingertwo = 0.0
        self.fingerthree = 0.0
        self.preshape = 0.0
        self.method = "pose"

    def isOpen(self):
        return 0 

    def objectInHand(self):
        return 0

    def sensedFingerPositions(self):

        return [sensed_finger_1, , ,sensed_preshape]
    # velocity: an array of 4,
    # positive speed moves finger inward,
    # and negative speed moves finger outward
    def setVelocity(self, velocity):
        now = rospy.get_rostime()
        currentTime = float(now.secs) + 1e-9*float(now.nsecs)
        regVelocity(velocity)
        self.fingerone = velocity[0]
        self.fingertwo = velocity[1]
        self.fingerthree = velocity[2]
        self.preshape = velocity[3]
        self.method = "velocity"

    def regVelocity(velocity):
        for i in range(len(velocity)):
            if velocity[i] > MAX_MOTOR_SPEED:
                print("command velocity for gripper is too fast, set to max velocity")
                velocity[i] = MAX_MOTOR_SPEED
            if velocity[i] < MIN_MOTOR_SPEED:
                print("command velocity for gripper is too fast, set to max velocity")
                velocity[i] = MIN_MOTOR_SPEED

    def calibrate(self):
        rospy.wait_for_service('/reflex_takktile2/calibrate_fingers')
        calibrate_fingers = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
        rospy.wait_for_service('/reflex_takktile2/calibrate_tactile')
        calibrate_tactile = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
        calibrate_fingers()
        calibrate_tactile()

    def set_threshold(self, x):
        set_tactile_threshold = rospy.ServiceProxy('/your_gripper/set_tactile_threshold', SetTactileThreshold)
        set_tactile_threshold(x)


    def stop(self):
        self.enable = False
        self.exit = True

    def shutDown(self):
        self.enable = False
        self.exit = True
        zero 

    def newState(self):
        return self.newStateFlag
 
    def markRead()
        newStateFlag = False