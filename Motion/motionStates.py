class LimbState:
    def __init__(self):
        self.sensedq = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.commandedq = []
        self.commandedqQueue = []
        self.senseddq = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.commandeddq = []
        self.senseddqQueue = []
        self.sensedWrench = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.gravityVector = [0,0,-9.81]

        self.commandSent = True
        self.commandType = 0 #0 is position, 1 is velocity
        self.commandQueue = False
        self.lastCommandQueueTime = 0.0
        #for kinematic to use...
        self.lastSensedq = []

class BaseState():
    def __init__(self):
        self.measuredPos = [0.0, 0.0, 0.0] #[x, y, yaw]
        self.measuredVel = [0.0, 0.0] #[v, w]
        self.commandedVel = [0.0, 0.0] #[v, w]
        self.commandedPath = [] #[(x0, y0, theta0), (x1, y1, theta1), ...] (list of waypoints)
