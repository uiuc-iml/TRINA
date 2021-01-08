import copy
import time
import numpy as np
from klampt.math import vectorops,so3,se3

class KinematicLimbController:
    def __init__(self, getConfig, getVelocity, setConfig, setVelocity, newState):
        """
        Parameters:
        -------------
        getConfig: Get limb config callback
        getVelocity: Get limb velocity callback
        setConfig: Set limb config callback
        setVelocity: Set limb velocity callback
        newState: newState update needed callback

        Note: All of these are just basically wrapping simulated_robot

        """
        self.moving = None
        self.getVelocity = getVelocity
        self.getConfig = getConfig
        self.setVelocity = setVelocity
        self.setConfig = setConfig
        self.getCurrentTime = None
        self.getWrench = lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.zeroFTSensor = lambda: None

        self.markRead = lambda: None
        self.newState = newState

        # Is this needed? idk
        self.start = lambda: True
        # This is needed.
        self.stop = lambda: True

        self.closeGripper = lambda: None
        self.openGripper = lambda: None

class Limb:
    def __init__(self, name, EE_link, active_dofs, wrench_transform_CB, controller):
        """
        Parameters:
        -------------
        name: "left" or "right"
        EE_link: End effector link (needed for some control loops)
        active_dofs: limb active dofs in config (needed for velocity control jacobian, collision checking (TODO)
        wrench_transform_CB: function to call to get the rotation of wrench (callback in case shoulder can move)
        controller: The limb controller

        """

        self.name = name
        self.EE_link = EE_link
        self.active_dofs = active_dofs
        self.wrench_transform_CB = wrench_transform_CB

        self.controller = controller
        self.enabled = controller is not None
        
        self.state = LimbState()

        if self.enabled:
            # Function pointer binding
            self.moving = controller.moving
            self.getVelocity = controller.getVelocity
            self.getConfig = controller.getConfig
            self.setVelocity = controller.setVelocity
            self.setConfig = controller.setConfig
            self.getCurrentTime = controller.getCurrentTime
            self.getWrench = controller.getWrench
            self.zeroFTSensor = controller.zeroFTSensor

            self.stop = controller.stop

            self.closeGripper = controller.closeGripper
            self.openGripper = controller.openGripper

            #self.markRead = controller.markRead
            #self.newState = controller.newState

    def start(self):
        """Start the limb controller.
        Parameters:
        -------------

        Return:
        -------------
        True on success, False on failure.
        """
        res = self.controller.start()
        time.sleep(1) #TODO: Do we need this sleep?
        if res == False:
            return False
        else:
            self.state.sensedq = self.getConfig()[0:6]
            self.state.senseddq = self.getVelocity()[0:6]
            self.state.sensedWrench = self.getWrench()
            return True

    def updateState(self):
        """Update the LimbState using info from the limb controller.
        Parameters:
        -------------

        Return:
        -------------
        """
        if self.controller.newState():
            self.state.sensedq = self.getConfig()[0:6]
            self.state.senseddq = self.getVelocity()[0:6]
            self.state.sensedWrench = self.getWrench()
            self.controller.markRead()

    def sensedEETransform(self, tool_center=None):
        """Return the transform of the tool position w.r.t. the base frame
        Parameters:
        -------------
        tool_center: Tool center (or none to use state tool center)

        Return:
        -------------
        (R,t)
        """
        # Promoting internal consistency maybe...?????
        if tool_center is None:
            tool_center = self.state.toolCenter

        T = self.EE_link.getTransform()
        return (T[0],vectorops.add(T[1],so3.apply(T[0],tool_center)))

    def sensedEEVelocity(self, local_pt = [0,0,0]):
        """Return the EE translational and rotational velocity  w.r.t. the base Frame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
        position_J = np.array(self.EE_link.getJacobian(local_pt))[:, self.active_dofs]
        EE_vel = np.dot(position_J,self.state.senseddq).tolist()
        return (EE_vel[3:],EE_vel[:3])

    def sensedEEWrench(self,frame='global',tool_center=None):
        """
        Parameters:
        ------------------
        frame:  'global' or 'local'
        tool_center: Tool center (or none to use state tool center)

        Return:
        ------------------
        wrench: list of 6 floats, expressed either in global or local EE coordinate, gravity of the attachement compensated for

        Note:
        The attachment weight to the ft sensor can be corrected by U5 directly
        """

        if tool_center is None:
            tool_center = self.state.toolCenter

        wrench_raw = self.state.sensedWrench #this wrench is expressed in the robot base frame
        (R,t) = self.sensedEETransform(tool_center=[0, 0, 0]) #current EE R in global frame
        R_base_global = self.wrench_transform_CB() # Transform from global to robot base, rotation

        if frame == 'global':
            (_,t_tool) = self.sensedEETransform(tool_center)

            wrench = list(so3.apply(R_base_global,wrench_raw[0:3]) + so3.apply(R_base_global,wrench_raw[3:6]))
            r = vectorops.sub(t_tool,t)
            torque = vectorops.cross(r,wrench[0:3])
            return wrench[0:3] + vectorops.sub(wrench[3:6],torque)

        elif frame == 'local':
            R_global_base = so3.inv(R_base_global) # Transform from robot base frame to global
            R_EE_base = so3.mul(R_global_base,R)

            # TODO: Check my math (Jing-Chen)
            wrench = list(so3.apply(so3.inv(R_EE_base),wrench_raw[0:3]) + so3.apply(so3.inv(R_EE_base),wrench_raw[3:6]))
            #r = tool_center
            torque = vectorops.cross(tool_center,wrench[0:3])
            return wrench[0:3] + vectorops.sub(wrench[3:6],torque)

        else:
            print(f"Invalid argument for frame, must be global or local, got {frame}")
            return [0]*6

    def pause(self):
        self.controller.pause()

    def resume(self):
        self.controller.resume()

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
        self.startTransform = ([1,0,0,0,1,0,0,0,1],[0,0,0]) # NOTE: This is unused......
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
        # I - delta_t A and its LU decomposition to do backward euler
        # A = [[0, 1], [-Minv K, -Minv B]]
        self.A = None
        self.LU = None
        #mass transform and velocity
        self.T_mass = []
        self.x_dot_mass = []
        #counter for how many iterations have passed
        self.counter = 1
        #range for ignoring the wrench readings
        self.deadband = [0]*6
        self.prev_wrench = np.array([0]*6)
        self.increaseB = False
        self.last_p_time = time.monotonic()

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
        self.set_mode_reset()   # Sets commandType to 0, which is needed.

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

        self.commandType = 1 # 0 is position, 1 is velocity, 2 is ramped velocity
        self.commandSent = True
        self.rampDuration = 0.0


        #used by the kinematic mode
        self.v_queue = []
        self.w_queue = []
        self.queue_idx = 0

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
        self.sensedVelocity = [0]*2
        self.commandedPosition = [0.0,0.0]
        self.newCommand = False
