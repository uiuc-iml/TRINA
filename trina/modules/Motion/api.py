from trina import jarvis
import numpy as np

class MotionAPI(jarvis.APILayer):
    #handled by superclass
    #def __init__(self,*args,**kwargs):
    #   APILayer.__init__(self,*args,**kwargs)

    def mode(self):
        """Returns the Motion Server mode, either 'Kinematic' or 'Physical'."""
        return self._redisGet(['ROBOT_INFO','Mode'])

    def activeComponents(self):
        """Returns a list of active components as queried by the Motion Server."""
        return self._redisGet(['ROBOT_INFO','Components'])
        
    def setPosition(self, q):
        """set the position of the entire robot

        Parameter:
        ---------------
        q: a merged list of joint positions, in the order of torso,base,left limb, right limb, left gripper...
        """
        raise NotImplementedError("TODO")
        return 0
    
    def mirror_arm_config(self,config):
        """given the Klampt config of the left or right arm, return the other

        Paremeters:
        ---------------
        A list of 6 doubles. Limb configuration.

        Return:
        ---------------
        A list of 6 doubles. Limb configuration.
        """
        RConfig = []
        RConfig.append(-config[0])
        RConfig.append(-config[1]-math.pi)
        RConfig.append(-config[2])
        RConfig.append(-config[3]+math.pi)
        RConfig.append(-config[4])
        RConfig.append(-config[5])
        return RConfig

    def setLeftLimbPosition(self, q):
        """Set the left limb joint positions, and the limb moves as fast as possible

        This will clear the motion queue.

        Parameter:
        --------------
        q: a list of 6 doubles. The desired joint positions.
        """
        return self._moduleCommand('setLeftLimbPosition', q)

    def setRightLimbPosition(self, q):
        """Set the right limb joint positions, and the limb moves as fast as possible

        This will clear the motion queue.

        Parameter:
        --------------
        q: a list of 6 doubles. The desired joint positions.
        """
        return self._moduleCommand('setRightLimbPosition', q)

    def setLeftLimbPositionLinear(self, q, duration):
        """Set Left limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameters:
        ----------------
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
        return self._moduleCommand('setLeftLimbPositionLinear', q, duration)

    def setRightLimbPositionLinear(self, q, duration):
        """Set right limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameters:
        ----------------
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
        return self._moduleCommand('setRightLimbPositionLinear', q, duration)

    def setVelocity(self, qdot):
        """set the velocity of the entire robot, under development rn
        """
        return self._moduleCommand('setVelocity',qdot)

    def setLeftLimbVelocity(self, qdot):
        """Set the left limb joint velocities

        Parameter:
        ----------------
        qdot: a list of 6 doubles. Joint velocities
        """
        return self._moduleCommand('setLeftLimbVelocity',qdot)

    def setRightLimbVelocity(self, qdot):
        """Set the right limb joint velocities

        Parameter:
        ----------------
        qdot: a list of 6 doubles. Joint velocities
        """
        return self._moduleCommand('setRightLimbVelocity', qdot)

    def setLeftEEInertialTransform(self, Ttarget, duration):
        """Set the trasform of the arm w.r.t. the base frame, movement complsetLeftLimbCo
        """
        return self._moduleCommand('setLeftEEInertialTransform', Ttarget, duration)

    def setLeftEEVelocity(self, v, tool=None):
        """Set the end-effect cartesian velocity, in the base frame.

        Implemented using position control and IK. Will keep moving until infeasible.
        TODO: implement collision detection

        Parameter:
        --------------
        v: A list of 6 doubled. v[0:3] is the desired cartesian position velocities and v[3:6] is the desired rotational velocity

        """
        if tool is None:
            tool = [0, 0, 0]
        return self._moduleCommand('setLeftEEVelocity', v, tool)

    def setRightEEInertialTransform(self, Ttarget, duration):
        """Set the trasform of the arm w.r.t. the base frame, movement complete in a certain amount of time

        This current version assmumes that the torso is at zero position.
        #TODO: implement version with torso not at zero.

        Parameter:
        ---------------
        Ttarget: A klampt rigid transform (R,t). R is a column major form of a rotation matrix. t is a 3-element list
        duration: double. The duration of the movement
        """
        return self._moduleCommand('setRightEEInertialTransform', Ttarget, duration)

    def setRightEEVelocity(self, v, tool=None):
        """Set the end-effect cartesian velocity, in the base frame.

        Implemented using position control and IK. Will keep moving until infeasible.
        TODO: implement collision detection

        Parameter:
        --------------
        v: A list of 6 doubled. v[0:3] is the desired cartesian position velocities and v[3:6] is the desired rotational velocity

        """
        if tool is None:
            tool = [0, 0, 0]
        return self._moduleCommand('setRightEEVelocity', v, tool)

    def setBaseTargetPosition(self, q, vel):
        """Set the local target position of the base.

        The base constructs a path to go to the desired position, following the desired speed along the path
        Parameter:
        ---------------
        q: a list of 3 doubles. The desired x,y position and rotation.
        Vel: double. Desired speed along the path.
        """
        return self._moduleCommand('setBaseTargetPosition', q, vel)

    def setBaseVelocity(self, q):
        """Set the velocity of the base relative to the local base frame

        Parameter:
        ---------------
        q: a list of 2 doubles. The linear and rotational velocites.
        """
        return self._moduleCommand('setBaseVelocity', q)

    def setTorsoTargetPosition(self, q):
        """Set the torso target position.

        Moves to the target as fast as possible.

        Parameter:
        --------------
        q: a list of 2 doubles. The lift and tilt positions.
        """
        return self._moduleCommand('setTorsoTargetPosition', q)

    def setLeftGripperPosition(self, position):
        """Set the position of the gripper. Moves as fast as possible.

        Parameters:
        -----------------
        position: a list of 4 doubles, the angles of finger 1,2 , the angle of the thumb,
            the rotation of finger 1&2 (they rotate together)
        """
        return self._moduleCommand('setLeftGripperPosition', position)

    def setLeftGripperVelocity(self, velocity):
        """Set the velocity of the gripper. Moves as fast as possible.
        #TODO
        ###Under development
        """
        return self._moduleCommand('setLeftGripperVelocity', velocity)

    def isStarted(self):
        """Return whether the robot has started

        Return:
        ------------
        bool
        """
        return self._redisGet(['ROBOT_INFO','Started'])

    def isShutDown(self):
        """Return whether the robot is shutdown

        Return:
        ------------
        bool
        """
        return self._redisGet(['ROBOT_INFO','Shutdown'])

    def moving(self):
        """Returns true if the robot is currently moving.

        Return:
        ------------
        bool
        """     
        return self._redisGet(['ROBOT_INFO','Moving'])

    def stopMotion(self):
        """Pause the motion of the robot, starting from the next control loop.

        This is not shutdown. Just a pause.
        """
        return self._moduleCommand('stopMotion')

    def resumeMotion(self):
        """Unpause the robot.

        After unpausing, the robot is still stationery until some new commands is added
        """
        return self._moduleCommand('resumeMotion')

    def sensedLeftEEWrench(self,frame = 'global'):
        """
        Parameters:
        ------------------
        frame:  1 = 'global' or 0 = 'local'
        Return:
        ------------------
        wrench: list of 6 floats, expressed either in global or local EE coordinate, gravity of the attachement compensated for

        Note:
        The attachment weight to the ft sensor can be corrected by U5 directly
        """
        return self._redisGet(['ROBOT_STATE','EEWrench','LeftArm'] + [frame])

    def sensedRightEEWrench(self,frame = 'global'):
        """
        Parameters:
        ------------------
        frame:  1 = 'global' or 0 = 'local'
        Return:
        ------------------
        wrench: list of 6 floats, expressed either in global or local EE coordinate

        """
        return self._redisGet(['ROBOT_STATE','EEWrench','RightArm'] + [frame])

    def openLeftRobotiqGripper(self):
        """ Open the parallel gripper or release the vacuum gripper. This gripper is connected to the arm.
        """
        return self._moduleCommand('openLeftRobotiqGripper')

    def closeLeftRobotiqGripper(self):
        """ close the parallel gripper or start the vacuum gripper. This gripper is connected to the arm.
        """
        return self._moduleCommand('closeLeftRobotiqGripper')

    def openRightRobotiqGripper(self):
        """ Open the parallel gripper or release the vacuum gripper. This gripper is connected to the arm.
        """
        return self._moduleCommand('openRightRobotiqGripper')

    def closeRightRobotiqGripper(self):
        """ close the parallel gripper or start the vacuum gripper. This gripper is connected to the arm.
        """
        return self._moduleCommand('closeRightRobotiqGripper')

    def setLeftEETransformImpedance(self,Tg,K,M,B,x_dot_g = [0]*6,deadband = [0]*6):
        """Set the target transform of the EE in the global frame. The EE will follow a linear trajectory in the cartesian space to the target transform.
        The EE will behave like a spring-mass-damper system attached to the target transform. The user will need to supply the elasticity matrix, the damping matrix,
        and the inertia matrix

        Parameters:
        -------------
        Tg: target transform of the EE, in Klampt format
        K: a 6x6 numpy 2D array. The elasticity matrix, this should be a diagonal matrix. The ordering is that the first 3 diagonal entries are for translations.
        B: a 6x6 numpy 2D array. The damping matrix.
        M: a 6x6 numpy 2D array. The inertia matrix.
        x_dot_g: list of 6 elements. The optional desired EE velocity
        deadband: list of 6 elements. This is the range for ignoring the wrench readings (kind of like "deadband")

        Return:
        -------------
        None
        """
        return self._moduleCommand('setLeftEETransformImpedance',
         (Tg),(K),(M),(B),(x_dot_g),(deadband))

    def setRightEETransformImpedance(self,Tg,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6):
        """Set the target transform of the EE in the global frame. The EE will follow a linear trajectory in the cartesian space to the target transform.
        The EE will behave like a spring-mass-damper system attached to the target transform. The user will need to supply the elasticity matrix, the damping matrix,
        and the inertia matrix

        Parameters:
        -------------
        Tg: target transform of the EE, in Klampt format
        K: a 6x6 numpy 2D array. The elasticity matrix, this should be a diagonal matrix. The ordering is that the first 3 diagonal entries are for translations.
        B: a 6x6 numpy 2D array. The damping matrix.
        M: a 6x6 numpy 2D array. The inertia matrix.
        x_dot_g: list of 6 elements. The optional desired EE velocity
        deadband: list of 6 elements. This is the range for ignoring the wrench readings (kind of like "deadband")

        Return:
        -------------
        None
        """
        return self._moduleCommand('setRightEETransformImpedance',
         (Tg),(K),(M),(B),(x_dot_g),(deadband))

    def setLeftLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6):
        """Set the target position of the limb. The EE will follow a linear trajectory in the cartesian space to the target transform.
        The EE will behave like a spring-mass-damper system attached to the target transform. The user will need to supply the elasticity matrix, the damping matrix,
        and the inertia matrix

        Parameters:
        -------------
        q: target positin of the limb
        K: a 6x6 numpy 2D array. The elasticity matrix, this should be a diagonal matrix. The ordering is that the first 3 diagonal entries are for translations.
        B: a 6x6 numpy 2D array. The damping matrix.
        M: a 6x6 numpy 2D array. The inertia matrix.
        x_dot_g: list of 6 elements. The optional desired EE velocity
        deadband: list of 6 elements. This is the range for ignoring the wrench readings (kind of like "deadband")

        Return:
        -------------
        None
        """
        return self._moduleCommand('setLeftLimbPositionImpedance',
         q,(K),(M),(B),(x_dot_g),(deadband))

    def setRightLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6):
        """Set the target position of the limb. The EE will follow a linear trajectory in the cartesian space to the target transform.
        The EE will behave like a spring-mass-damper system attached to the target transform. The user will need to supply the elasticity matrix, the damping matrix,
        and the inertia matrix

        Parameters:
        -------------
        q: target positin of the limb
        K: a 6x6 numpy 2D array. The elasticity matrix, this should be a diagonal matrix. The ordering is that the first 3 diagonal entries are for translations.
        B: a 6x6 numpy 2D array. The damping matrix.
        M: a 6x6 numpy 2D array. The inertia matrix.
        x_dot_g: list of 6 elements. The optional desired EE velocity
        deadband: list of 6 elements. This is the range for ignoring the wrench readings (kind of like "deadband")

        Return:
        -------------
        None
        """
        return self._moduleCommand('setRightLimbPositionImpedance',
                q,(K),(M),(B),(x_dot_g),(deadband))     

    def cartesianDriveFail(self):
        """ Return if cartedian drive has failed or not

        Return:
        ----------------
        bool
        """
        return self._redisGet(["ROBOT_INFO","CartesianDrive"])

    def sensedLeftEEVelocity(self, local_pt=[0, 0, 0]):
        """Return the EE translational and rotational velocity  w.r.t. the base DataFrame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
        return self._redisGet(["ROBOT_STATE","VelocityEE","LeftArm"])

    def sensedRightEEVelocity(self, local_pt=[0, 0, 0]):
        """Return the EE translational and rotational velocity  w.r.t. the base DataFrame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
        return self._redisGet(["ROBOT_STATE","VelocityEE","RightArm"])

    def getComponents(self):
        """ Return robot component dictionary

        Return:
        ----------------
        dict
        """
        return self._redisGet(["ROBOT_INFO","Components"])

    def getKlamptSensedPosition(self):
        """Return the entire sensed Klampt position, in Klampt format.
        """
        return self._redisGet(["ROBOT_STATE","KlamptSensedPos"])

    def getKlamptCommandedPosition(self):
        """Return the entire commanded position, in Klampt format. The base returns velocity instead
        """
        return self._redisGet(["ROBOT_STATE","KlamptCommandPos"])

    def sensedBaseVelocity(self):
        """Returns the current base velocity

        Return:
        -----------
        A list of 2 doubles. Linear and Rotational velocities.
        """
        return self._redisGet(["ROBOT_STATE","Velocity","Base"])

    def sensedLeftLimbVelocity(self):
        """ Return the current limb joint velocities

        Return:
        ---------------
        A list of 6 doubles. The joint velocities.
        """
        return self._redisGet(["ROBOT_STATE","Velocity","LeftArm"])

    def sensedRightLimbVelocity(self):
        """ Return the current limb joint velocities

        Return:
        ---------------
        A list of 6 doubles. The joint velocities.
        """
        return self._redisGet(["ROBOT_STATE","Velocity","RightArm"])

    def sensedBasePosition(self):
        """Returns the current base position. Zero position is the position when the base is started.

        Return:
        -------------
        A list of 3 doubles. Position and rotation.

        """
        return self._redisGet(["ROBOT_STATE","Position","Base"])

    def sensedTorsoPosition(self):
        """Returns the current torso position

        Return:
        -------------
        A list of 2 doubles. The positions.
        """
        return self._redisGet(["ROBOT_STATE","Position","Torso"])

    def sensedLeftEETransform(self):
        """Return the transform w.r.t. the base frame

        Return:
        -------------
        (R,t)
        """
        return self._redisGet(["ROBOT_STATE","PositionEE","LeftArm"])

    def sensedRightEETransform(self):
        """Return the transform w.r.t. the base frame

        Return:
        -------------
        (R,t)
        """
        return self._redisGet(["ROBOT_STATE","PositionEE","RightArm"])

    def sensedLeftGripperPosition(self):
        """Return the current positions of the fingers.
        #TODO
        ###Under development
        """
        return self._redisGet(["ROBOT_STATE","Position","LeftGripper"])

    def sensedRobotq(self):
        """Return the Robotq position.
        """
        return self._redisGet(["ROBOT_STATE","Position","Robotq"])

    def sensedRightLimbPosition(self):
        """The current joint positions of the right limb

        Return:
        --------------
        A list of 6 doubles. The limb configuration.
        """
        return self._redisGet(["ROBOT_STATE","Position","RightArm"])

    def sensedLeftLimbPosition(self):
        """The current joint positions of the left limb

        Return:
        --------------
        A list of 6 doubles. The limb configuration.
        """
        return self._redisGet(["ROBOT_STATE","Position","LeftArm"])
