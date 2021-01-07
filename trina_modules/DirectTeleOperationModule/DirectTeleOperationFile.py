import time,math
# from klampt import vis
# from klampt import WorldModel
# from klampt.model.trajectory import Trajectory
import threading
# from Motion.motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import pickle
from pdb import set_trace
from numpy import linalg as LA
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import datetime
import csv
from threading import Thread
import sys
import json
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback
import signal
from klampt.math import so3, so2, se3, vectorops
from Motion import TRINAConfig

from robot_v2 import robot as controller_listen

from enum import Enum

DEGREE_2_RADIAN = 2.0*math.pi/180.0

class ControllerMode(Enum):
    ABSOLUTE=0
    CLUTCHING=1

robot_ip = 'http://localhost:8080'


ws_port = 1234

#model_name = "Motion/data/TRINA_world_seed.xml"

roomname = "The Lobby"
zonename = "BasicExamples"
userId=0
roomId=-1
is_closed=0

mode="Physical" # UNUSED FOR ANYTHING

class TeleopArmState:
	def __init__(self, side, active, gripper_active, set_limb_position_linear, 
			transform_sensor, transform_setter,
			proportional_controller, impedance_controller,
			open_robotiq_gripper, close_robotiq_gripper):
		self.side = side
		self.joystick = side + "Controller"
		self.active = active
		self.gripper_active = gripper_active
		self.gripper_open = True
		self.init_pos = None
		self.cur_pos = None
		self.setLimbPositionLinear = set_limb_position_linear
		self.sensedEETransform = transform_sensor
		self.setEEInertialTransform = transform_setter
		self.setEEVelocity = proportional_controller
		self.setEETransformImpedance = impedance_controller
		self.openRobotiqGripper = open_robotiq_gripper
		self.closeRobotiqGripper = close_robotiq_gripper

		self.teleoperationState = 0

class DirectTeleOperation:
	"""
	mode: Robot (physical) or simulation (absolute)
	controller_mode: Absolute or Clutching
	"""
	def __init__(self,Jarvis = None, debugging = False, mode = mode, controller_mode = ControllerMode.CLUTCHING):
		self.mode = mode
		self.status = 'idle' #states are " idle, active"
		self.state = 'idle' #states are " idle, active"
		self.init_UI_state = {}
		self.dt = 0.025
		self.infoLoop_rate = 0.05
		self.max_arm_speed = 0.5
		self.robot = Jarvis
		self.components =  ['head','base','left_limb','right_limb', 'left_gripper', 'right_gripper']
		#self.robot.getComponents()
		left_limb_active = ('left_limb' in self.components)
		left_gripper_active = ('left_gripper' in self.components)
		self.left_limb = TeleopArmState("left", left_limb_active, 
			left_gripper_active, 
			self.robot.setLeftLimbPositionLinear,
			self.robot.sensedLeftEETransform, 
			self.robot.setLeftEEInertialTransform,
			self.robot.setLeftEEVelocity, 
			self.robot.setLeftEETransformImpedance,
			self.robot.openLeftRobotiqGripper,
			self.robot.closeLeftRobotiqGripper)

		right_limb_active = ('right_limb' in self.components)
		right_gripper_active = ('right_gripper' in self.components)
		self.right_limb = TeleopArmState("right", right_limb_active, 
			right_gripper_active, 
			self.robot.setRightLimbPositionLinear,
			self.robot.sensedRightEETransform, 
			self.robot.setRightEEInertialTransform,
			self.robot.setRightEEVelocity, 
			self.robot.setRightEETransformImpedance,
			self.robot.openRightRobotiqGripper,
			self.robot.closeRightRobotiqGripper)

		self.base_active = ('base' in self.components)
		self.torso_active = ('torso' in self.components)
		self.head_active = ('head' in self.components)
		self.temp_robot_telemetry = {'leftArm':[0,0,0,0,0,0],'rightArm':[0,0,0,0,0,0]}

		self.max_disp = 0.125 # To tune

		self.K = np.diag([200.0, 200.0, 200.0, 10.0, 10.0, 10.0])

		self.M = 1*np.eye(6)#*5.0
		self.M[3,3] = 0.25
		self.M[4,4] = 0.25
		self.M[5,5] = 0.25

		self.B = 2.0*np.sqrt(4.0*np.dot(self.M,self.K))
		self.B[3:6,3:6] = 0.75*self.B[3:6,3:6]
		# self.B[3:6,3:6] = self.B[3:6,3:6]*2.0
		# self.M = np.diag((2,2,2,1,1,1))
		# self.B = np.sqrt(32 * self.K *ABSOLUTE self.M)
		self.K = self.K.tolist()
		self.M = self.M.tolist()
		self.B = self.B.tolist()

		self.tool = np.array([0.0,0.0,0.0], dtype=np.float64)
		self.sensitivity = 1.0
		self.last_sens_button_state = False
		self.sens_state = 0

		# 0 - Go home
		# 1 - Set controller home
		# 2 - Move hands
		# Folded into arm objects.
		# self.teleoperationState = 0self
		# Absolute - Home position, everything relative to that
		# Clutching - Relative to when you last hit clutch
		self.controller_mode = controller_mode

		self.UI_state = {}
		self.init_headset_orientation = {}
		self.init_headset_rotation = {} #x,y position
		self.panLimits = {"center": 180, "min":90, "max":270} #head limits
		self.tiltLimits = {"center": 180, "min":130, "max":230} #head limits
		self.startup = True
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl+c)

		stateRecieverThread = threading.Thread(target=self._serveStateReceiver)
		main_thread = threading.Thread(target = self._infoLoop)
		controller_thread = Process(target = controller_listen.listen, daemon=True)
		stateRecieverThread.start()
		main_thread.start()
		controller_thread.start()
		# time.sleep(5)

	def sigint_handler(self, signum, frame):
		""" Catch Ctrl+C tp shutdown the api,
			there are bugs with using sigint_handler.. not used rn.

		"""
		assert(signum == signal.SIGINT)
		print("SIGINT caught...shutting down the api!")

	def return_threads(self):
		return [self._serveStateReceiver, self._infoLoop]

	def return_processes(self):
		return []

	def _infoLoop(self):
		while(True):
			self.robot.log_health()
			loop_start_time = time.time()
			status = self.robot.getActivityStatus()

			if(status == 'active'):
				if(self.status == 'idle'):
					print('\n\n\n\n starting up Direct-Tele Operation Module! \n\n\n\n\n')
					self.status = 'active'
					self.state = 'active'

			elif(status == 'idle'):
				if self.status == 'active':
					# self.state = 'idle'
					# self.status = 'idle'self
					# DEBUGGING ONLY - SET TO IDLE AFTER TESTING!!!
					self.state = 'active'
					self.status = 'active'
			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.infoLoop_rate:
				time.sleep(self.infoLoop_rate)
			else:
				time.sleep(0.001)

	def _serveStateReceiver(self):
		# self.setRobotToDefault()
		time.sleep(3)
		# while(True):
		if((self.startup == True) and (self.robot.getUIState() !=0)):
			print('started the initial values for the variables')
			self.init_UI_state = self.robot.getUIState()
			self.startup = False
			if(self.left_limb.active):
				self.left_limb.init_pos = self.left_limb.sensedEETransform()
			if(self.right_limb.active):
				self.right_limb.init_pos = self.right_limb.sensedEETransform()
			self.init_headset_orientation = self.treat_headset_orientation(self.init_UI_state['headSetPositionState']['deviceRotation'])
			self.init_headset_rotation = self.init_UI_state['headSetPositionState']['deviceRotation']
		while(True):
			if self.state == 'idle':
				# print("_serveStateReceiver: idling")
				pass
			elif self.state == 'active':
				# print('_serveStateReceiver:active')
				self.last_time = time.time()
				if(self.left_limb.active):
					self.left_limb.cur_pos = self.left_limb.sensedEETransform()
				if(self.right_limb.active):
					self.right_limb.cur_pos = self.right_limb.sensedEETransform()
				self.UI_state = self.robot.getUIState()
				self.UIStateLogic()
				time.sleep(self.dt)


	def setRobotToDefault(self):
		home_duration = 4
		if self.left_limb.active:
			self.left_limb.setLimbPositionLinear(TRINAConfig.left_untucked_config, home_duration)
		if self.right_limb.active:
			self.right_limb.setLimbPositionLinear(TRINAConfig.right_untucked_config, home_duration)


	def UIStateLogic(self):
		if(type(self.UI_state)!= int):
			if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
				print("Robot Home")
				self.setRobotToDefault()
				self.left_limb.teleoperationState = 1
				self.right_limb.teleoperationState = 1
			if (
				not self.UI_state["controllerButtonState"]["rightController"]
				["press"][0] and self.last_sens_button_state
			):
				# Released the right controller button
				self.sens_state = (self.sens_state + 1) % 2
				if self.sens_state == 0:
					self.sensitivity = 1.0
				else:
					self.sensitivity = 0.25
				self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])
				for limb in (self.left_limb, self.right_limb):
					self.init_UI_state["controllerPositionState"][limb.joystick]["controllerPosition"] = (
						self.UI_state["controllerPositionState"][limb.joystick]["controllerPosition"])
					self.init_UI_state["controllerPositionState"][limb.joystick]['controllerRotation'] = (
						self.UI_state["controllerPositionState"][limb.joystick]['controllerRotation'])
					limb.init_pos = limb.sensedEETransform()
			self.last_sens_button_state = (self.UI_state
				["controllerButtonState"]["rightController"]["press"][0])
			if self.controller_mode == ControllerMode.ABSOLUTE:
				if (self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True and self.left_limb.teleoperationState == 1):
					print('\n\n\n\n resetting UI initial state \n\n\n\n\n')
					self.init_UI_state = self.UI_state
					self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])
					self.init_headset_rotation = self.init_UI_state['headSetPositionState']['deviceRotation']
					
					for limb in (self.left_limb, self.right_limb):
						limb.init_pos = limb.sensedEETransform()
						limb.teleoperationState = 2

			elif self.controller_mode == ControllerMode.CLUTCHING:
				for limb in (self.left_limb, self.right_limb):
					if self.UI_state["controllerButtonState"][limb.joystick]["squeeze"][1] > 0.5:
						if limb.teleoperationState == 3 or limb.teleoperationState == 1:
							self.init_UI_state["controllerPositionState"][limb.joystick]["controllerPosition"] = (
								self.UI_state["controllerPositionState"][limb.joystick]["controllerPosition"])
							self.init_UI_state["controllerPositionState"][limb.joystick]['controllerRotation'] = (
								self.UI_state["controllerPositionState"][limb.joystick]['controllerRotation'])

							self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])
							self.init_headset_rotation = self.init_UI_state['headSetPositionState']['deviceRotation']
							
							limb.init_pos = limb.sensedEETransform(self.tool.tolist())

							# print("BEGIN CLUTCHING, ZERO!")

							limb.teleoperationState = 2

					elif limb.teleoperationState == 2:
						# Released button, go to idle
						limb.teleoperationState = 3
						#limb.setEEVelocity([0,0,0,0,0,0], tool = self.tool.tolist())
						limb.setEETransformImpedance(
							limb.sensedEETransform(tool_center=self.tool.tolist()),
							self.K, self.M, [[10*x for x in a] for a in self.B],
							tool_center=self.tool.tolist())
						# limb.setEETransformImpedance(
						# 	limb.sensedEETransform(tool_center=self.tool.tolist()),
						# 	np.zeros((6,6)).tolist(), self.M, [[100*x for x in a] for a in self.B],
						# 	tool_center=self.tool.tolist())
			print("UI state Logic")
			if(self.base_active):
				self.baseControl()
			
			if(self.head_active):
				self.headControl()	

			self.control('impedance')

		
	
	def headControl(self):
		def mod180(x):
			while x >= 180:
				x = x - 360
			while x < -180:
				x = x + 360
			return x

		def limitTo(x,min,max):
			if x > max:
				return max
			elif x < min:
				return min
			return x

		def to_rotvec(orientation):
			# print("quat",orientation)
			right_handed_rotvec =  np.array([-orientation[2],orientation[0],-orientation[1],-(np.pi/180)*orientation[3]])

			partial_rotation = R.from_rotvec(right_handed_rotvec[:3]*right_handed_rotvec[3])
			# we then get its equivalent row, pitch and yaw
			rpy = partial_rotation.as_euler('ZYX',degrees=True)
			rpy2 = partial_rotation.as_euler('YZX',degrees=True)
			return {"y":rpy[0],"x":rpy2[0]}

		orientation = to_rotvec(self.UI_state['headSetPositionState']['deviceRotation'])
		init_orientation = to_rotvec(self.init_headset_rotation)
		panAngle = limitTo((self.panLimits["center"] - mod180(orientation["y"] - init_orientation["y"])), self.panLimits["min"], self.panLimits["max"])
		tiltAngle = limitTo((self.tiltLimits["center"] - mod180(orientation["x"] - init_orientation["x"])), self.tiltLimits["min"], self.tiltLimits["max"])
		
		try:
			self.robot.setHeadPosition([panAngle*DEGREE_2_RADIAN, tiltAngle*DEGREE_2_RADIAN])
		except Exception as err:
			print("Error: {0}".format(err))
			pass

	def baseControl(self):
		'''controlling base movement'''
		base_velocity = [0.1*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.1*(-self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][0])]
		curr_velocity = np.array(self.robot.sensedBaseVelocity())
		base_velocity_vec = np.array(base_velocity)
		# if the commanded velocity differs from the actual velocity:
		if((curr_velocity-base_velocity_vec).sum()!= 0):
			try:
				self.robot.setBaseVelocity(base_velocity)
			except:
				print("setBaseVelocity not successful")
				pass

	def control(self, mode):
		if self.left_limb.active and self.left_limb.teleoperationState == 2:
			self.controlArm(self.left_limb, mode)
			self.temp_robot_telemetry['leftArm'] = self.robot.sensedLeftLimbPosition()
		if self.right_limb.active and self.right_limb.teleoperationState == 2:
			self.controlArm(self.right_limb, mode)
			self.temp_robot_telemetry['rightArm'] = self.robot.sensedRightLimbPosition()

		if (self.mode == 'Physical'):
			for limb in [self.left_limb, self.right_limb]:
				if limb.gripper_active:
					closed_value = self.UI_state["controllerButtonState"][limb.joystick]["squeeze"][0]
					if(closed_value > 0 and limb.gripper_open):
						limb.closeRobotiqGripper()
						limb.gripper_open = False
					elif(closed_value <= 0 and not limb.gripper_open):
						limb.gripper_open = True
						limb.openRobotiqGripper()

		self.robot.addRobotTelemetry(self.temp_robot_telemetry)

	"""
	Control an arm based on UI state and other jazz. Might lock it.

	limb: Arm object.
	mode: One of position, velocity, or impedance.
	"""
	def controlArm(self, limb, mode):
		assert (mode in ['position', 'velocity', 'impedance']), "Invalid mode"
		if self.UI_state["controllerButtonState"][limb.joystick]["squeeze"][1] > 0.5:
			RR_final, RT_final, curr_transform = self.getTargetEETransform(limb)
			target_transform = (RR_final, RT_final)
			actual_dt = 1 * self.dt
			gain = 2.0
			error = vectorops.mul(se3.error((RR_final, RT_final), curr_transform), gain)
			# Set EE Velocity wants (v, w), error gives (w, v)
			error_t = error[3:] + error[:3]
			if mode == 'position':
				limb.setEEInertialTransform(target_transform, actual_dt)
			elif mode == 'velocity':
				limb.setEEVelocity(error_t, tool = self.tool.tolist())
			elif mode == 'impedance':
				limb.setEETransformImpedance(target_transform, self.K, self.M, self.B, tool_center=self.tool.tolist())
		else:
			limb.setEETransformImpedance(limb.sensedEETransform(), self.K, self.M, [[4*x for x in a] for a in self.B])

	def getTargetEETransform(self, limb):
		"""Get the transform of the end effector attached to the `side` arm
		in the frame of the <base?> of the robot.

		LT_cw_cc => Left Translational matrix from Controller World to
			Controller Current
		RR_rw_rh => Right Rotational matrix from Robot World to Robot Home
		RR_rw_rh_T => Right Rotational matrix from Robot World to
			Robot Home Transpose
		R_cw_rw  => Rotational matrix from Controller World to Robot World

		cc------controller current
		rc------robot current

		cw------controller world
		rw------robot world

		ch------controller home
		rh------robot home

		Parameters
		----------------
		limb: limb object
			Which arm to compute transform for.

		Returns
		----------------
		tuple: (list, list, tuple)
			Desired rotation transform and translation transform for the
			requested arm, (R,t) tuple representing the current transform of
			the requested arm
		"""
		R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
		[RR_rw_rh,RT_rw_rh] = limb.init_pos
		curr_transform = limb.sensedEETransform()

		RT_rw_rh = np.array(RT_rw_rh)
		RT_cw_cc = np.array(self.UI_state["controllerPositionState"]
			[limb.joystick]["controllerPosition"])
		RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"]
			[limb.joystick]["controllerPosition"])
		RT_final = ( RT_rw_rh
			+ (np.matmul(
			np.matmul(self.init_headset_orientation.as_dcm(),R_cw_rw)
			,(RT_cw_cc - RT_cw_ch).T)) ).tolist()
		RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))

		init_quat = np.array(
			self.init_UI_state["controllerPositionState"]
				[limb.joystick]['controllerRotation'])

		R_cw_rw = R.from_dcm(R_cw_rw)

		# world_adjustment_matrix = R_cw_rw
		world_adjustment_matrix = R.from_dcm(np.eye(3))
		init_angle = (np.pi/180)*init_quat[3]

		right_handed_init_vec = np.array(
			[-init_quat[2],init_quat[0],-init_quat[1]]) * (-init_angle)

		#transform it to right handed:
		curr_quat = np.array(self.UI_state["controllerPositionState"][limb.joystick]
			['controllerRotation'])
		curr_angle = (np.pi/180)*curr_quat[3]
		right_handed_curr_vec = np.array(
			[-curr_quat[2],curr_quat[0],-curr_quat[1]]) * (-curr_angle)

		RR_cw_ch = R.from_rotvec(right_handed_init_vec)
		RR_cw_ch_T = RR_cw_ch.inv()
		RR_cw_cc = R.from_rotvec(right_handed_curr_vec)
		RR_ch_cc = (self.init_headset_orientation * (RR_cw_ch_T * RR_cw_cc)
			* self.init_headset_orientation.inv())

		RR_final = (RR_rw_rh*RR_ch_cc).as_dcm().flatten().tolist()
		t_final = list(se3.interpolate(limb.init_pos, (RR_final, RT_final),
			self.sensitivity))
		dist = se3.distance(t_final, curr_transform, Rweight=0.0)
		if dist > self.max_disp:
			w = self.max_disp / dist
			t_final[1] = ((1 - w) * np.array(curr_transform[1]) 
				+ w * np.array(t_final[1])).tolist()
		return t_final[0], t_final[1], curr_transform

	def treat_headset_orientation(self,headset_orientation):
		"""
		input: Rotvec of the headset position
		output: Rotation Matrix for the headset that ignores pitch and roll
		"""
		#first we turn the input into a right_handed rotvec
		right_handed_rotvec =  np.array([-headset_orientation[2],headset_orientation[0],-headset_orientation[1],-(np.pi/180)*headset_orientation[3]])

		partial_rotation = R.from_rotvec(right_handed_rotvec[:3]*right_handed_rotvec[3])
		# we then get its equivalent row, pitch and yaw
		rpy = partial_rotation.as_euler('ZYX')
		# we then ignore its roll and pitch in unity
		rotation_final = R.from_euler('ZYX',[rpy[0],0,0])

		#so3 version:
		# partial_rotation = so3.from_rotation_vector((right_handed_rotvec[:3]*right_handed_rotvec[3]).tolist())
		# # we then get its equivalent roll, pitch and yaw
		# rpy = so3.rpy(partial_rotation)
		# # # we then ignore its roll and pitch in unity
		# rotation_final = so3.from_rpy([rpy[0],0,0])

		# # print('Treated initial matrix - here is the transform')
		# # print(rotation_final.as_dcm())

		# return so3.matrix(rotation_final)
		return rotation_final



if __name__ == "__main__" :
	my_controller = DirectTeleOperation()
