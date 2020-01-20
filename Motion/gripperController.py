from __future__ import print_function
from __future__ import division
import roslib
import rospy
import time
import threading



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
    def __init__(self):
        self.command_pub = rospy.Publisher('/reflex_takktile2/command', Command, queue_size=5)
        self.pos_pub = rospy.Publisher('/reflex_takktile2/command_position', PoseCommand, queue_size=5)
        self.vel_pub = rospy.Publisher('/reflex_takktile2/command_velocity', VelocityCommand, queue_size=5)
        self.sub = rospy.Subscriber('/reflex_takktile2/hand_state', Hand, self.callback)

        self.dt = 1.0/10.0

        self.sense_finger_set = [0.0, 0.0, 0.0, 0.0]
        self.command_finger_set = [0.0, 0.0, 0.0, 0.0]

        self.command_type = None
        self.contact = False

        self.enable = False
        self.exit = False
        self.stop = False
        self.new_state = False




    def start(self):
        self.enable = True
        rospy.init_node('gripperController')
        controlThread = threading.Thread(target = self.controlLoop)
        controlThread.start()

    def controlLoop(self):

        now = rospy.get_rostime()

        self.lastStateTime = float(now.secs) + 1e-9*float(now.nsecs) #get the ros time for each robot state msg
        rate = rospy.Rate(1.0/self.dt)

        # self.calibrate()
        while not rospy.is_shutdown() and (not self.exit):
            print(self.sense_finger_set)

            if self.stop:
                self.pos_pub.publish(VelocityCommand(f1 = 0.0, f2 = 0.0, f3 = 0.0, preshape = 0.0))
            elif self.command_type == "pose":
                self.pos_pub.publish(PoseCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))
                #self.command_type = None
            elif self.command_type == "velocity":
                self.vel_pub.publish(VelocityCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))
                # self.command_type = None

            rate.sleep()
    def callback(self, data):
        # self.lastStateTime = float(data.header.stamp.secs)+1e-9*float(data.header.stamp.nsecs)
        self.sense_finger_set[0] = data.motor[0].joint_angle
        self.sense_finger_set[1] = data.motor[1].joint_angle
        self.sense_finger_set[2] = data.motor[2].joint_angle
        self.sense_finger_set[3] = data.motor[3].joint_angle
        for finger in data.finger:
            for sensor in finger.contact:
                self.contact |= sensor
                if self.contact == True:
                    break
            if self.contact == True:
                break
        self.new_state = True



    def regPose(self, position):
        for i in range(len(position)):
            if position[i] > MAX_MOTOR_TRAVEL:
                print("command position for gripper is too far, set to max position")
                position[i] = MAX_MOTOR_TRAVEL
            if position[i] < MIN_MOTOR_TRAVEL:
                print("command position for gripper is too far, set to max position")
                position[i] = MIN_MOTOR_TRAVEL

    def regVelocity(self, velocity):
        for i in range(len(velocity)):
            if velocity[i] > MAX_MOTOR_SPEED:
                print("command velocity for gripper is too fast, set to max velocity")
                velocity[i] = MAX_MOTOR_SPEED
            if velocity[i] < MIN_MOTOR_SPEED:
                print("command velocity for gripper is too fast, set to max velocity")
                velocity[i] = MIN_MOTOR_SPEED



    def setPose(self, position):
        # now = rospy.get_rostime()
        # currentTime = float(now.secs) + 1e-9*float(now.nsecs)
        self.regPose(position)
        self.command_finger_set[0] = position[0]
        self.command_finger_set[1] = position[1]
        self.command_finger_set[2] = position[2]
        self.command_finger_set[3] = position[3]
        self.command_type = "pose"

    def setVelocity(self, velocity):
        # now = rospy.get_rostime()
        # currentTime = float(now.secs) + 1e-9*float(now.nsecs)
        self.regVelocity(velocity)
        self.command_finger_set[0] = velocity[0]
        self.command_finger_set[0] = velocity[1]
        self.command_finger_set[0] = velocity[2]
        self.command_finger_set[0] = velocity[3]
        self.command_type = "velocity"

    def close(self):
        self.command_finger_set[0] = 3.0
        self.command_finger_set[1] = 3.0
        self.command_finger_set[2] = 3.0
        self.command_finger_set[3] = 1.0
        self.command_type = "pose"

    def open(self):
        self.command_finger_set[0] = 0.0
        self.command_finger_set[1] = 0.0
        self.command_finger_set[2] = 0.0
        self.command_finger_set[3] = 1.0
        self.command_type = "pose"


####################################
###  probably won't be used now  ###
####################################
    def calibrate(self):
        rospy.wait_for_service('/reflex_takktile2/calibrate_fingers')
        calibrate_fingers = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
        rospy.wait_for_service('/reflex_takktile2/calibrate_tactile')
        calibrate_tactile = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
        calibrate_fingers()
        calibrate_tactile()

####################################
###  probably won't be used now  ###
####################################
    def set_threshold(self, x):
        set_tactile_threshold = rospy.ServiceProxy('/your_gripper/set_tactile_threshold', SetTactileThreshold)
        set_tactile_threshold(x)



    def is_open(self):
        print("is_open(): ", self.sense_finger_set)
        if self.sense_finger_set[0] == 0.0 and self.sense_finger_set[1] == 0.0 and self.sense_finger_set[2] == 0.0:
            return 1
        else:
            return 0

    def contact_with_object(self):
        return self.contact

    def sensed_finger_positions(self):
        return self.sense_finger_set



    def stop(self):
        self.enable = False
        self.stop = True

    def resume(self):
        self.enable = True
        self.stop = False

    def shutDown(self):
        self.enable = False
        self.exit = True

    def moving(self):
        if self.enable:
            return True
        else:
            return False

    def mark_read(self):
        self.new_state = False

    def new_state(self):
        return self.new_state

def hand_state_cb(data):
    global hand_state
    hand_state = data
    #print(data)

if __name__ == "__main__":
    con = GripperController()
    con.start()
    while True:
        con.close()
        rospy.sleep(2)
        con.open()
        rospy.sleep(2)
        print(con.is_open())
    # con.setPose([0.0, 0.0, 0.0, 0.0])
    # con.setPose([1.0, 1.0, 1.0, 1.0])
    # rospy.init_node('ExampleHandNode')
    #
    # # Services can automatically call hand calibration
    # calibrate_fingers = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
    # calibrate_tactile = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
    #
    # # Services can set tactile thresholds and enable tactile stops
    # enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile2/enable_tactile_stops', Empty)
    # disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile2/disable_tactile_stops', Empty)
    # set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile2/set_tactile_threshold', SetTactileThreshold)
    #
    # # This collection of publishers can be used to command the hand
    # command_pub = rospy.Publisher('/reflex_takktile2/command', Command, queue_size=1)
    # pos_pub = rospy.Publisher('/reflex_takktile2/command_position', PoseCommand, queue_size=1)
    # vel_pub = rospy.Publisher('/reflex_takktile2/command_velocity', VelocityCommand, queue_size=1)
    #
    # rospy.Subscriber('/reflex_takktile2/hand_state', Hand, hand_state_cb)
    # # raw_input("== When ready to calibrate the hand, press [Enter]\n")
    # # calibrate_fingers()
    # # raw_input("...\n")
    # print("2")
    # rate = rospy.Rate(20)
    # while not rospy.is_shutdown():
    #     pos_pub.publish(PoseCommand(2.7, 2.7, 2.7, 0.0))
    #     rate.sleep()

    # con.setPose((1.0, 1.0,1.0,1.0))
