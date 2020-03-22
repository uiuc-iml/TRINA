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

        ##cartesian velocity drive
        self.cartesianDrive = False
        self.cartesianDriveV = [0,0,0]
        self.cartesianDriveW = [0,0,0]
        self.startTransform = ([1,0,0,0,1,0,0,0,1],[0,0,0])
        self.driveTransform = ([1,0,0,0,1,0,0,0,1],[0,0,0])
        self.driveSpeedAdjustment = 1.0
        self.cartesianMode = 0 # 0 means both translation and rotation, 2 only rotation, 1 only position
        self.toolCenter = [0,0,0]
        
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
