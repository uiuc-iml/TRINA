import csv
import os
import inspect
from klampt import io
import uuid
from multiprocessing import Pool, TimeoutError
import traceback
from reem.datatypes import KeyValueStore
from reem.connection import RedisInterface
from threading import Thread
import numpy as np
from multiprocessing import Process, Manager, Pipe
import json
import time
import math
import datetime
import threading
import sys
import time
import redis

# from Modules import *
# import command_server


class Jarvis:

	def __init__(self, name, sensor_module=[], trina_queue=None, host="localhost"):
		self.interface = RedisInterface(host=host)
		self.interface.initialize()
		self.server = KeyValueStore(self.interface)
		if(trina_queue == None):
			self.trina_queue = TrinaQueue(str(name))
		else:
			self.trina_queue = trina_queue
		self.name = str(name)
		self.server['ACTIVITY_STATUS'][self.name] = str('idle')
		self.sensor_module = sensor_module
		self.server['ROBOT_COMMAND'][self.name] = []

    def motionMode(self):
        """Returns the Motion Server mode, either 'Kinematic' or 'Physical'."""
        return self.server['ROBOT_INFO']['Mode'].read()

    def activeComponents(self):
        """Returns a list of active components as queried by the Motion Server."""
        return self.server['ROBOT_INFO']['Components'].read()
		
	def setPosition(self, q):
		"""set the position of the entire robot

        Parameter:
        ---------------
        q: a merged list of joint positions, in the order of torso,base,left limb, right limb, left gripper...
        """
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
		command = self.send_command('self.robot.setLeftLimbPosition', str(q))

		return 0

	def setRightLimbPosition(self, q):
		"""Set the right limb joint positions, and the limb moves as fast as possible

        This will clear the motion queue.

        Parameter:
        --------------
        q: a list of 6 doubles. The desired joint positions.
        """
		command = self.send_command('self.robot.setRightLimbPosition', str(q))
		return 0

	def setLeftLimbPositionLinear(self, q, duration):
		"""Set Left limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameters:
        ----------------
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
		command = self.send_command('self.robot.setLeftLimbPositionLinear', str(q), str(duration))
		return 0

	def setRightLimbPositionLinear(self, q, duration):
		"""Set right limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameters:
        ----------------
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
		command = self.send_command('self.robot.setRightLimbPositionLinear', str(q), str(duration))
		return 0

	def setVelocity(self, qdot):
		"""set the velocity of the entire robot, under development rn
        """
		command = self.send_command('self.robot.setVelocity', str(qdot))

	def setLeftLimbVelocity(self, qdot):
		"""Set the left limb joint velocities

        Parameter:
        ----------------
        qdot: a list of 6 doubles. Joint velocities
        """
		command = self.send_command('self.robot.setLeftLimbVelocity', str(qdot))

	def setRightLimbVelocity(self, qdot):
		"""Set the right limb joint velocities

        Parameter:
        ----------------
        qdot: a list of 6 doubles. Joint velocities
        """
		command = self.send_command('self.robot.setRightLimbVelocity', str(qdot))

	def setLeftEEInertialTransform(self, Ttarget, duration):
		"""Set the trasform of the arm w.r.t. the base frame, movement complsetLeftLimbCo
        """
		command = self.send_command(
			'self.robot.setLeftEEInertialTransform', str(Ttarget), str(duration))

	def setLeftEEVelocity(self, v, tool):
		"""Set the end-effect cartesian velocity, in the base frame.

        Implemented using position control and IK. Will keep moving until infeasible.
        TODO: implement collision detection

        Parameter:
        --------------
        v: A list of 6 doubled. v[0:3] is the desired cartesian position velocities and v[3:6] is the desired rotational velocity

        """
		if not tool:
			tool = [0, 0, 0]
		command = self.send_command(
			'self.robot.setLeftEEVelocity', str(v), str(tool))

	def setRightEEInertialTransform(self, Ttarget, duration):
		"""Set the trasform of the arm w.r.t. the base frame, movement complete in a certain amount of time

        This current version assmumes that the torso is at zero position.
        #TODO: implement version with torso not at zero.

        Parameter:
        ---------------
        Ttarget: A klampt rigid transform (R,t). R is a column major form of a rotation matrix. t is a 3-element list
        duration: double. The duration of the movement
        """
		command = self.send_command(
			'self.robot.setRightEEInertialTransform', str(Ttarget), str(duration))

	def setRightEEVelocity(self, v, tool):
		"""Set the end-effect cartesian velocity, in the base frame.

        Implemented using position control and IK. Will keep moving until infeasible.
        TODO: implement collision detection

        Parameter:
        --------------
        v: A list of 6 doubled. v[0:3] is the desired cartesian position velocities and v[3:6] is the desired rotational velocity

        """
		if not tool:
			tool = [0, 0, 0]
		command = self.send_command(
			'self.robot.setRightEEVelocity', str(v), str(tool))

	def setBaseTargetPosition(self, q, vel):
		"""Set the local target position of the base.

        The base constructs a path to go to the desired position, following the desired speed along the path
        Parameter:
        ---------------
        q: a list of 3 doubles. The desired x,y position and rotation.
        Vel: double. Desired speed along the path.
        """
		command = self.send_command(
			'self.robot.setBaseTargetPosition', str(q), str(vel))

	def setBaseVelocity(self, q):
		"""Set the velocity of the base relative to the local base frame

        Parameter:
        ---------------
        q: a list of 2 doubles. The linear and rotational velocites.
        """
		command = self.send_command('self.robot.setBaseVelocity', str(q))

	def setTorsoTargetPosition(self, q):
		"""Set the torso target position.

        Moves to the target as fast as possible.

        Parameter:
        --------------
        q: a list of 2 doubles. The lift and tilt positions.
        """
		command = self.send_command('self.robot.setTorsoTargetPosition', str(q))

	def setLeftGripperPosition(self, position):
		"""Set the position of the gripper. Moves as fast as possible.

        Parameters:
        -----------------
        position: a list of 4 doubles, the angles of finger 1,2 , the angle of the thumb,
            the rotation of finger 1&2 (they rotate together)
        """
		command = self.send_command(
			'self.robot.setLeftGripperPosition', str(position))

	def setLeftGripperVelocity(self, velocity):
		"""Set the velocity of the gripper. Moves as fast as possible.
        #TODO
        ###Under development
        """
		command = self.send_command(
			'self.robot.setLeftGripperVelocity', str(velocity))

	def isStarted(self):
		"""Return whether the robot has started

        Return:
        ------------
        bool
        """
		return self.server['ROBOT_INFO']['Started'].read()

	def isShutDown(self):
		"""Return whether the robot is shutdown

        Return:
        ------------
        bool
        """
		return self.server['ROBOT_INFO']['Shutdown'].read()

	def moving(self):
		"""Returns true if the robot is currently moving.

        Return:
        ------------
        bool
        """		
		return self.server['ROBOT_INFO']['Moving'].read()

	def mode(self):
		"""Returns the current mode. "Kinematic" or "Physical"

        Return:
        ------------
        string
        """
		return self.server['ROBOT_INFO']['MODE'].read()

	def stopMotion(self):
		"""Pause the motion of the robot, starting from the next control loop.

        This is not shutdown. Just a pause.
        """
		command = self.send_command('self.robot.stopMotion')

	def resumeMotion(self):
		"""Unpause the robot.

        After unpausing, the robot is still stationery until some new commands is added
        """
		command = self.send_command('self.robot.resumeMotion')

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
		if frame == 'global':
			return self.server['ROBOT_STATE']['EEWrench']['LeftArm']['global'].read()
		elif frame == 'local':
			return self.server['ROBOT_STATE']['EEWrench']['LeftArm']['local'].read()

	def sensedRightEEWrench(self,frame = 'global'):
		"""
        Parameters:
        ------------------
        frame:  1 = 'global' or 0 = 'local'
        Return:
        ------------------
        wrench: list of 6 floats, expressed either in global or local EE coordinate

        """
		if frame == 'global':
			return self.server['ROBOT_STATE']['EEWrench']['RightArm']['global'].read()
		elif frame == 'local':
			return self.server['ROBOT_STATE']['EEWrench']['RightArm']['local'].read()

	def openLeftRobotiqGripper(self):
		""" Open the parallel gripper or release the vacuum gripper. This gripper is connected to the arm.
        """
		command = self.send_command('self.robot.openLeftRobotiqGripper')

	def closeLeftRobotiqGripper(self):
		""" close the parallel gripper or start the vacuum gripper. This gripper is connected to the arm.
        """
		command = self.send_command('self.robot.closeLeftRobotiqGripper')

	def openRightRobotiqGripper(self):
		""" Open the parallel gripper or release the vacuum gripper. This gripper is connected to the arm.
        """
		command = self.send_command('self.robot.openRightRobotiqGripper')

	def closeRightRobotiqGripper(self):
		""" close the parallel gripper or start the vacuum gripper. This gripper is connected to the arm.
        """
		command = self.send_command('self.robot.closeRightRobotiqGripper')

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
		command = self.send_command('self.robot.setLeftEETransformImpedance',
		 str(Tg),str(K),str(M),str(B),str(x_dot_g),str(deadband))

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
		command = self.send_command('self.robot.setRightEETransformImpedance',
		 str(Tg),str(K),str(M),str(B),str(x_dot_g),str(deadband))

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
		command = self.send_command('self.robot.setLeftLimbPositionImpedance',
		 str(q),str(K),str(M),str(B),str(x_dot_g),str(deadband))

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
		command = self.send_command('self.robot.setRightLimbPositionImpedance',
				str(q),str(K),str(M),str(B),str(x_dot_g),str(deadband))		

	def getWorld(self):
		""" Return the simulated robot

        Return:
        -------------
        The Klampt world of the simulated robot.
        """
		return self.server["WORLD"].read()

	def cartesianDriveFail(self):
		""" Return if cartedian drive has failed or not

        Return:
        ----------------
        bool
        """
		return self.server["ROBOT_INFO"]["CartesianDrive"].read()

	def sensedLeftEEVelocity(self, local_pt=[0, 0, 0]):
		"""Return the EE translational and rotational velocity  w.r.t. the base DataFrame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
		return self.server["ROBOT_STATE"]["VelocityEE"]["LeftArm"].read()

	def sensedRightEEVelocity(self, local_pt=[0, 0, 0]):
		"""Return the EE translational and rotational velocity  w.r.t. the base DataFrame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
		return self.server["ROBOT_STATE"]["VelocityEE"]["RightArm"].read()

	def getUIState(self):
		""" Return UI state dictionary

        Return:
        ----------------
        dict
        """
		return self.server["UI_STATE"].read()

	def getRobotState(self):
		""" Return robot state dictionary

        Return:
        ----------------
        dict
        """
		return self.server["ROBOT_STATE"].read()

	def getComponents(self):
		""" Return robot component dictionary

        Return:
        ----------------
        dict
        """
		return self.server["ROBOT_INFO"]["Components"].read()

	def addRobotTelemetry(self, value):
		""" Add UI robot telemetry value
        """
		self.server["robotTelemetry"] = value

	def getKlamptSensedPosition(self):
		"""Return the entire sensed Klampt position, in Klampt format.
        """
		return self.server["ROBOT_STATE"]["KlamptSensedPos"].read()

	def getKlamptCommandedPosition(self):
		"""Return the entire commanded position, in Klampt format. The base returns velocity instead
        """
		return self.server["ROBOT_STATE"]["KlamptCommandPos"].read()

	def sensedBaseVelocity(self):
		"""Returns the current base velocity

        Return:
        -----------
        A list of 2 doubles. Linear and Rotational velocities.
        """
		return self.server["ROBOT_STATE"]["Velocity"]["Base"].read()

	def sensedLeftLimbVelocity(self):
		""" Return the current limb joint velocities

        Return:
        ---------------
        A list of 6 doubles. The joint velocities.
        """
		return self.server["ROBOT_STATE"]["Velocity"]["LeftArm"].read()

	def sensedRightLimbVelocity(self):
		""" Return the current limb joint velocities

        Return:
        ---------------
        A list of 6 doubles. The joint velocities.
        """
		return self.server["ROBOT_STATE"]["Velocity"]["RightArm"].read()

	def sensedBasePosition(self):
		"""Returns the current base position. Zero position is the position when the base is started.

        Return:
        -------------
        A list of 3 doubles. Position and rotation.

        """
		return self.server["ROBOT_STATE"]["Position"]["Base"].read()

	def sensedTorsoPosition(self):
		"""Returns the current torso position

        Return:
        -------------
        A list of 2 doubles. The positions.
        """
		return self.server["ROBOT_STATE"]["Position"]["Torso"].read()

	def sensedLeftEETransform(self):
		"""Return the transform w.r.t. the base frame

        Return:
        -------------
        (R,t)
        """
		return self.server["ROBOT_STATE"]["PositionEE"]["LeftArm"].read()

	def sensedRightEETransform(self):
		"""Return the transform w.r.t. the base frame

        Return:
        -------------
        (R,t)
        """
		return self.server["ROBOT_STATE"]["PositionEE"]["RightArm"].read()

	def sensedLeftGripperPosition(self):
		"""Return the current positions of the fingers.
        #TODO
        ###Under development
        """
		return self.server["ROBOT_STATE"]["Position"]["LeftGripper"].read()

	def sensedRobotq(self):
		"""Return the Robotq position.
        """
		return self.server["ROBOT_STATE"]["Position"]["Robotq"].read()

	def sensedRightLimbPosition(self):
		"""The current joint positions of the right limb

        Return:
        --------------
        A list of 6 doubles. The limb configuration.
        """
		return self.server["ROBOT_STATE"]["Position"]["RightArm"].read()

	def sensedLeftLimbPosition(self):
		"""The current joint positions of the left limb

        Return:
        --------------
        A list of 6 doubles. The limb configuration.
        """
		return self.server["ROBOT_STATE"]["Position"]["LeftArm"].read()

	def getActivityStatus(self):
		"""Returns the module activity 

        Return:
        --------------
		dict
        """
		return self.server['ACTIVITY_STATUS'][self.name].read()

	def get_point_clouds(self):
		"""Returns the point clouds from all cameras

        Return:
        --------------
		point clouds
        """
		return self.sensor_module.get_point_clouds()

	def get_rgbd_images(self):
		"""Returns the RGBD cameras from all cameras

        Return:
        --------------
		images
        """
		return self.sensor_module.get_rgbd_images()

	def send_command(self, command, *args):
		final_string = str(command) + '('
		for index, arg in enumerate(args):
			if(index != len(args)-1):
				final_string += '{},'
			else:
				final_string += '{}'
		final_string = (final_string + ')')
		final_string = final_string.format(*args)
		self.trina_queue.push(final_string)
		print('sending ',final_string)

	def log_health(self, status=True):
		"""Add the health log of the module
		"""
		self.server["HEALTH_LOG"][self.name] = [status, time.time()]

	def changeActivityStatus(self, to_activate, to_deactivate=[]):
		"""Changes the status of the module
        """
		command = self.send_command(
			'self.switch_module_activity', str(to_activate), str(to_deactivate))

	def getTrinaTime(self):
		"""Returns the trina time
        """
		return self.server['TRINA_TIME'].read()

############################# All Mighty divider between motion and UI ###############################
	def sendRayClickUI(self):

		"""once this function is called, the UI will ask the user to click twice on the map, and sending back 
		2 ray objects according to the user clicks. first one for destination, second one for calibration
		return:
			id: (str) id for ui feedback
		blocking?:
			no
		"""
		id = '$' + uuid.uuid1().hex
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED': False, 'MSG': ''}
		# ask the user to click on a destination in the map, returns 2 rays in reem
		self._do_rpc({'funcName': 'getRayClick', 'args': {'id': str(id)}})
		return id
		
	def getRayClickUI(self,id):
		"""get the feedback of Ray Click of id. 
		return:
			'NOT READY': (str) if the msg is not ready
			or
			{
				'FIRST_RAY': {'destination': [-0.8490072256426063,-0.2846905378876157,-0.4451269801347757],
							'source': [12.653596500469428, 1.6440497080649081, 5.851982763380186]},
				'SECOND_RAY': {'destination': [-0.8590257360168888,-0.20712234383654582,-0.46816142466493127],
							'source': [12.653596500469428, 1.6440497080649081, 5.851982763380186]}
			}
		blocking?:
			no
		"""
		return self.getFeedback(id)

	def sendAndGetRayClickUI(self):
		"""once this function is called, the UI will ask the user to click twice on the map, and sending back 
		2 ray objects according to the user clicks. first one for destination, second one for calibration
		
		return:
			
			{
				'FIRST_RAY': {'destination': [-0.8490072256426063,-0.2846905378876157,-0.4451269801347757],
							'source': [12.653596500469428, 1.6440497080649081, 5.851982763380186]},
				'SECOND_RAY': {'destination': [-0.8590257360168888,-0.20712234383654582,-0.46816142466493127],
							'source': [12.653596500469428, 1.6440497080649081, 5.851982763380186]}
			}
		blocking?:
			yes
		"""
		id = '$' + uuid.uuid1().hex
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED': False, 'MSG': ''}
		# ask the user to click on a destination in the map, returns 2 rays in reem
		self._do_rpc({'funcName': 'getRayClick', 'args': {'id': str(id)}})
		reply = self.checkFeedback(id)
		return reply

	def addTextUI(self, name, text, color, size):
		"""add text to specfified location on UI screen. 
		args:
			name: (str) id for the text object
			text: (str) content you wish to add
			color: (list) rgb value [0,0,0]
			size: (int) font size
		return:
			name: (str) the name/id the user gave 
		blocking?:
			no
		"""
		self._do_rpc({'funcName': 'addText', 'args': {
						'name': name, 'color': color, 'size': size,  'text': text}})
		return name

	def sendConfirmationUI(self,title,text):
		"""once this function is called, the UI will display a confimation window with specified title and text, 
		
		return:
			id: (str) id for ui feedback
		blocking?:
			no
		"""
		id = '$' + uuid.uuid1().hex
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED': False, 'MSG': ''}
		self._do_rpc({'funcName': 'addConfirmation', 'args': {
						'id': str(id), 'title': title, 'text': text}})
		return id
		
	def getConfirmationUI(self,id):
		"""get the feedback of Confirmation Window of id. 
		return:
			'NOT READY': (str) if the msg is not ready
			or
			(str) 'YES' or 'NO' if msg is ready
		blocking?:
			no
		"""
		return self.getFeedback(id)


	def sendAndGetConfirmationUI(self,title,text):
		"""once this function is called, the UI will display a confimation window with specified title and text, 
			a string of 'YES' or "NO" is returned
		args:
			text: (str) content you wish to add
			title: (str) window title
		return:
			(str) 'YES' or 'NO'
		blocking?:
			yes
		"""
		id = '$' + uuid.uuid1().hex
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED': False, 'MSG': ''}
		self._do_rpc({'funcName': 'addConfirmation', 'args': {
						'id': str(id), 'title': title, 'text': text}})
		reply = self.checkFeedback(id)
		return reply

	def sendTrajectoryUI(self,trajectory,animate = False):
		"""send a trajectory to UI, UI will add the path preview and animate? the robot ghost immediately for only once 
		
		args:
			trajectory: (klampt obj) the traj calculated
			animate: (bool) if user wants to animate the path
		return:
			nothing
		blocking?:
			no
		"""
		trajectory = io.loader.toJson(trajectory, 'Trajectory')
		self._do_rpc({'funcName': 'sendTrajectory', 'args': {
						'trajectory': trajectory, 'animate': animate}})
		return

	def addButtonUI(self, name, text):
		"""add a button to the UI window
		args:
			name: (str)  id for the button object
			text: (str) button label text
		return:
			name: the id user gave
		
		blocking?:
			no
		"""
		id = '$'+ name
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':False, 'MSG':''}
		self._do_rpc({'funcName':'addButton','args':{'name':name, 'text':text}})
		return name

	def getButtonClickUI(self,name):
		"""returns True if button with specified name is clicked
		args:
			name: (str) id for the button object
		return:
			(bool) True or False
		
		blocking?:
			no
		"""
		id = '$' + name
		reply = self.getFeedback(id)
		if reply:
			self.server['UI_FEEDBACK'][str(id)] = {
				'REPLIED': True, 'MSG': False}
		return reply

	def addPromptUI(self,title,text):
		id = '$'+ uuid.uuid1().hex
		# TODO
		return  id

	def addInputBoxUI(self,title,text,fields):
		id = '$'+ uuid.uuid1().hex
		# TODO
		return id

	def getSaveConfigSignalUI(self):
		if not self.server['UI_FEEDBACK']['move-to-save-signal']['REPLIED'].read():
			return 'NOT READY'
		else:
			name =  self.server['UI_FEEDBACK']['move-to-save-signal']['MSG'].read()
			self.server['UI_FEEDBACK']['move-to-save-signal']['REPLIED'] = False
			return True, name

	def getLoadConfigSignalUI(self):
		if not self.server['UI_FEEDBACK']['move-to-load-signal']['REPLIED'].read():
			return 'NOT READY'
		else:
			name =  self.server['UI_FEEDBACK']['move-to-load-signal']['MSG'].read()
			self.server['UI_FEEDBACK']['move-to-load-signal']['REPLIED'] = False
			return True, name

	def checkFeedback(self,id):
		while not self.server['UI_FEEDBACK'][str(id)]['REPLIED'].read():
			continue
		return self.server['UI_FEEDBACK'][str(id)]['MSG'].read()

	def getFeedback(self,id):
		if not self.server['UI_FEEDBACK'][str(id)]['REPLIED'].read():
			return 'NOT READY'
		return self.server['UI_FEEDBACK'][str(id)]['MSG'].read()

	def _do_rpc(self,msg):
		msg["from"] = self.name
		commandQueue = self.server["UI_END_COMMAND"].read()
		commandQueue.append(msg)
		self.server["UI_END_COMMAND"] = commandQueue
		print("commandQueue", commandQueue)
		time.sleep(0.0001)
		
# extra trina queue class:

class TrinaQueue(object):
	def __init__(self,key, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
	def push(self,item):
		self.r.rpush(self.key,item)

if __name__=="__main__":
	server = Jarvis()
