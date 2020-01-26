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

        while not rospy.is_shutdown() and (not self.exit):
            # print(self.sense_finger_set)

            if self.stop:
                self.pos_pub.publish(PoseCommand(f1 = 0.0, f2 = 0.0, f3 = 0.0, preshape = 0.0))
            elif self.command_type == "pose":
                self.pos_pub.publish(PoseCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))
            elif self.command_type == "velocity":
                self.vel_pub.publish(VelocityCommand(f1 = self.command_finger_set[0], f2 = self.command_finger_set[1], f3 = self.command_finger_set[2], preshape = self.command_finger_set[3]))

            rate.sleep()


    def callback(self, data):
        # self.lastStateTime = float(data.header.stamp.secs)+1e-9*float(data.header.stamp.nsecs)
        self.sense_finger_set[0] = data.motor[0].joint_angle
        self.sense_finger_set[1] = data.motor[1].joint_angle
        self.sense_finger_set[2] = data.motor[2].joint_angle
        self.sense_finger_set[3] = data.motor[3].joint_angle

        self.contact = False
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
        self.regPose(position)
        self.command_finger_set[0] = position[0]
        self.command_finger_set[1] = position[1]
        self.command_finger_set[2] = position[2]
        self.command_finger_set[3] = position[3]
        self.command_type = "pose"

    def setVelocity(self, velocity):
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

    def fclose(self):
        self.close()
        time.sleep(2)

    def open(self):
        self.command_finger_set[0] = 0.0
        self.command_finger_set[1] = 0.0
        self.command_finger_set[2] = 0.0
        self.command_finger_set[3] = 0.0
        self.command_type = "pose"

    def fopen(self):
        self.open()
        time.sleep(2)



#Calibrate will calibrate the finger and the sensor in the same time.
#Calibration may move the fingers.
#Also, calibrate sensor requires no contact
    def calibrate(self):
        rospy.wait_for_service('/reflex_takktile2/calibrate_fingers')
        calibrate_fingers = rospy.ServiceProxy('/reflex_takktile2/calibrate_fingers', Empty)
        rospy.wait_for_service('/reflex_takktile2/calibrate_tactile')
        calibrate_tactile = rospy.ServiceProxy('/reflex_takktile2/calibrate_tactile', Empty)
        self.fopen()
        calibrate_fingers()
        calibrate_tactile()

####################################
###  probably won't be used now  ###
####################################
    def set_threshold(self, x):
        set_tactile_threshold = rospy.ServiceProxy('/your_gripper/set_tactile_threshold', SetTactileThreshold)
        set_tactile_threshold(x)



    def is_open(self):
        # print("is_open(): ", self.sense_finger_set)
        if self.sense_finger_set[0] - 0 < 0.1 and self.sense_finger_set[1] - 0 < 0.1 and self.sense_finger_set[2] - 0 < 0.1:
            return True
        else:
            return False

    def contact_with_object(self):
        # print(self.contact)
        return self.contact

    def sensed_finger_positions(self):
        return self.sense_finger_set



    def stop_process(self):
        self.enable = False
        self.stop = True

    def fstop_process(self):
        self.stop_process()
        time.sleep(2)

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

    def newState(self):
        return self.new_state

def hand_state_cb(data):
    global hand_state
    hand_state = data
    #print(data)

if __name__ == "__main__":
    con = GripperController()
    con.start()
    con.setPose([2,2,0,1])
    time.sleep(2)
    # while True:
    #     con.close()
    #     rospy.sleep(2)
    #     con.open()
    #     rospy.sleep(2)
    #     print(con.is_open())
    # time.sleep(3)
    # con.calibrate()
    #con.fclose()
    #con.fopen()
    # time.sleep(3)
    # con.open()
    # time.sleep(2)
    # # print(con.is_open())
    # # print(con.is_open())
    # con.calibrate()
    # while True:
    #     print(con.contact_with_object())
    # time.sleep(1)
    # con.fopen()
    # con.setPose([1.0, 1.0, 1.0, 1.0])
    # time.sleep(2)
    # con.fopen()
    # con.setPose([2.0, 1.5, 1.0, 1.5])
    # time.sleep(2)
    # con.fopen()
    # con.setPose([3.5, 3.5, 3.0, 2.0])
    # time.sleep(2)
    # con.fopen()
    # con.setPose([3.5, 3.5, 1.0, 2.5])
    # time.sleep(2)
    # con.fopen()
    # con.setPose([2.0, 2.0, 3.7, 3.0])
    # time.sleep(2)
    # con.fopen()
    # con.setPose([3.0, 3.0, 3.0, 3.5])
    # time.sleep(2)
    # con.fstop_process()
    # print(con.contact_with_object())
    # print(con.sensed_finger_positions())
    con.shutDown()
    print("shutdown called")
