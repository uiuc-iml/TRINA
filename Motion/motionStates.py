import copy
import time
import numpy as np

class LimbState:
    def __init__(self):
        self.sensedq = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.commandedq = []
        self.difference = []
        self.commandedqQueueStart = []
        self.senseddq = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.commandeddq = []
        self.senseddqQueue = []
        self.sensedWrench = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.gravityVector = [0,0,-9.81]

        self.commandSent = True
        self.commandType = 0 #0 is position, 1 is velocity
        self.commandQueue = False
        self.commandQueueTime = 0.0
        self.commandedQueueDuration = 0.0
        #for kinematic to use...
        self.lastSensedq = []

        #TODO: This is completely unused... forward declaring it in prep for python 3 transition (Jing-Chen)
        self.Xs = []

        ##cartesian velocity drive
        self.cartesianDrive = False
        self.cartesianDriveV = [0,0,0]
        self.cartesianDriveW = [0,0,0]
        self.startTransform = ([1,0,0,0,1,0,0,0,1],[0,0,0])
        self.driveTransform = ([1,0,0,0,1,0,0,0,1],[0,0,0])
        self.driveSpeedAdjustment = 1.0
        self.cartesianMode = 0 # 0 means both translation and rotation, 2 only rotation, 1 only position
        self.toolCenter = [0,0,0]

        ##handling impedance control
        self.impedanceControl = False
        #goal transform and velocity
        self.T_g = []
        self.x_dot_g = []
        self.K = []
        self.B = []
        self.Minv = []
        #mass transform and velocity
        self.T_mass = []
        self.x_dot_mass = []
        #counter for how many iterations have passed
        self.counter = 1
        #range for ignoring the wrench readings
        self.deadband = [0]*6
        self.prev_wrench = np.array([0]*6)
        self.increaseB = False

    def set_mode_reset(self):
        #self.commandSent = True
        self.commandedq = []
        self.commandeddq = []
        self.commandType = 0
        self.commandQueue = False
        self.commandedqQueue = []
        self.cartesianDrive = False
        self.impedanceControl = False
        self.commandQueueTime = 0.0
        self.commandedQueueDuration = 0.0
        self.difference = []
        self.commandedqQueueStart = []
        self.Xs = []
        self.increaseB = False

    def set_mode_position(self, position):
        self.set_mode_reset()

        self.commandSent = False
        self.commandedq = copy.deepcopy(position)

    def set_mode_commandqueue(self, difference, start, duration):
        self.set_mode_reset()

        self.commandSent = False
        self.difference = difference
        self.commandedqQueueStart = copy.deepcopy(start)
        self.commandQueue = True
        self.commandedQueueDuration = duration
        self.commandQueueTime = time.time()

    def set_mode_velocity(self, qdot):
        self.set_mode_reset()

        self.commandSent = False
        self.commandeddq = copy.deepcopy(qdot)
        self.commandType = 1


class BaseState():
    def __init__(self):
        self.measuredPos = [0.0, 0.0, 0.0] #[x, y, yaw]
        self.measuredVel = [0.0, 0.0] #[v, w]
        self.commandedVel = [0.0, 0.0] #[v, w]

        self.generatedPath = None
        self.pathFollowingVel = 0.0
        self.pathFollowingIdx = 0
        self.pathFollowingNumPoints = 0
        self.commandedTargetPosition = [] #[x, y, theta]

        self.commandType = 1 # 0 is position, 1 is velocity, 2 is path
        self.commandSent = True

class GripperState:
    def __init__(self):
        self.sense_finger_set = [0.0, 0.0, 0.0, 0.0] #finger 1 finger 2 finger 3 preshape
        self.command_finger_set = [0.0, 0.0, 0.0, 0.0]
        self.commandType = 0 #0 is position, 1 is velocity

class TorsoState:
    def __init__(self):
        self.measuredHeight = 0.0
        self.measuredTilt = 0.0
        self.commandedHeight = 0.0
        self.commandedTilt = 0.0

        self.commandSent = True
        self.leftLeg = 0
        self.rightLeg = 0


class HeadState:
    def __init__(self):
        self.sensedPosition = [0.0,0.0]
        self.commandedPosition = [0.0,0.0]
        self.newCommand = False
