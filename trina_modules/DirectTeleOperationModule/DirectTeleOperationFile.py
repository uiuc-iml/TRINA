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

from robot_v2 import robot as controller_listen

from enum import Enum

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
	def __init__(self, side, active, gripper_active, transform_sensor, transform_setter,
						proportional_controller, impedance_controller):
		self.side = side
		self.joystick = side + "Controller"
		self.active = active
		self.gripper_active = gripper_active
		self.init_pos = None
		self.cur_pos = None
		self.sensedEETransform = transform_sensor
		self.setEEInertialTransform = transform_setter
		self.setEEVelocity = proportional_controller
		self.setEETransformImpedance = impedance_controller

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
		self.components =  ['base','left_limb','right_limb', 'left_gripper']
		#self.robot.getComponents()
		left_limb_active = ('left_limb' in self.components)
		left_gripper_active = ('left_gripper' in self.components)
		self.left_limb = TeleopArmState("left", left_limb_active, left_gripper_active,
										self.robot.sensedLeftEETransform, self.robot.setLeftEEInertialTransform,
										self.robot.setLeftEEVelocity, self.robot.setLeftEETransformImpedance)

		right_limb_active = ('right_limb' in self.components)
		right_gripper_active = ('right_gripper' in self.components)
		self.right_limb = TeleopArmState("right", right_limb_active, right_gripper_active, 
										self.robot.sensedRightEETransform, self.robot.setRightEEInertialTransform,
										self.robot.setRightEEVelocity, self.robot.setRightEETransformImpedance)

		self.base_active = ('base' in self.components)
		self.torso_active = ('torso' in self.components)
		self.temp_robot_telemetry = {'leftArm':[0,0,0,0,0,0],'rightArm':[0,0,0,0,0,0]}

		self.K = np.array([[200.0,0.0,0.0,0.0,0.0,0.0],\
                [0.0,200.0,0.0,0.0,0.0,0.0],\
                [0.0,0.0,200.0,0.0,0.0,0.0],\
                [0.0,0.0,0.0,5.0,0.0,0.0],\
                [0.0,0.0,0.0,0.0,5.0,0.0],\
                [0.0,0.0,0.0,0.0,0.0,5.0]])

		self.M = np.eye(6)*5.0
		self.M[3,3] = 1.0
		self.M[4,4] = 1.0
		self.M[5,5] = 1.0

		self.B = 2.0*np.sqrt(4.0*np.dot(self.M,self.K))
		self.B[3:6,3:6] = self.B[3:6,3:6]*2.0
		# self.M = np.diag((2,2,2,1,1,1))
		# self.B = np.sqrt(32 * self.K *ABSOLUTE self.M)
		self.K = self.K.tolist()
		self.M = self.M.tolist()
		self.B = self.B.tolist()

		#TODO VERY VERY TEMPORARY @REMOVE - Jing-Chen
		self.tool = np.array([0.0,0.0,0.0], dtype=np.float64)
		self.tool_target = np.array([0.27,0.0,0.0])

		# 0 - Go home
		# 1 - Set controller home
		# 2 - Move hands
		# Folded into arm objects.
		# self.teleoperationState = 0self
		# Absolute - Home position, everything relative to that
		# Clutching - Relative to when you last hit clutch
		self.controller_mode = controller_mode

		time.sleep(5)
		self.UI_state = {}
		self.init_headset_orientation = {}
		self.startup = True
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl+c)

		stateRecieverThread = threading.Thread(target=self._serveStateReceiver)
		main_thread = threading.Thread(target = self._infoLoop)
		controller_thread = threading.Thread(target = controller_listen.listen)
		stateRecieverThread.start()
		main_thread.start()
		controller_thread.start()

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

		while(True):
			if self.state == 'idle':
				# print("_serveStateReceiver: idling")
				pass
			elif self.state == 'active':
				# print('_serveStateReceiver:active')
				self.last_time = time.time()
				if(self.left_limb.active):
					self.left_limb.cur_pos = self.right_limb.sensedEETransform()
				if(self.right_limb.active):
					self.right_limb.cur_pos = self.right_limb.sensedEETransform()
				self.UI_state = self.robot.getUIState()
				self.UIStateLogic()
				time.sleep(self.dt)


	def setRobotToDefault(self):
		# rightUntuckedRotation = np.array([
		# 	0.9996677819374474, -0.020138967102307673, 0.016085638324823164,
		# 	-0.025232828744241535, -0.6373999040791976, 0.7701199040626047,
		# 	-0.005256435087453611, -0.7702699424772351, -0.6376963114259705
		# ])
		rightUntuckedRotation = np.array([
			0, 0, -1,
			0, -1, 0,
			-1, 0, 0
		])
		rotzm90 = np.array([
			[0, 1, 0],
			[-1, 0, 0],
			[0, 0, 1],
		])
		oort = 1/np.sqrt(2)
		rotxm45 = np.array([
			[1,0, 0],
			[0,oort,oort],
			[0,-oort,oort]
		])
		# rightUntuckedRotation = np.matmul(rightUntuckedRotation.reshape(3,3),
			# rotxm45).flatten()
		# rightUntuckedRotation = np.matmul(rightUntuckedRotation.reshape(3,3),
		# 	rotzm90).flatten()
		#rightUntuckedTranslation = np.array([0.6410086795413383, -0.196298410887376, 0.8540173127153597])
		rightUntuckedTranslation = np.array([0.34,
			-0.296298410887376, 1.0540173127153597])
		# Looks like the y axis is the left-right axis.
		# Mirroring along y axis.
		mirror_reflect_R = np.array([
							 1, -1,  1,
							-1,  1, -1,
							 1, -1,  1,
						])
		mirror_reflect_T = np.array([1, -1, 1])
		# Element wise multiplication.
		leftUntuckedRotation = rightUntuckedRotation * mirror_reflect_R
		leftUntuckedTranslation = rightUntuckedTranslation * mirror_reflect_T

		# TODO: This is broken for two reasons:
		#   1. Linear move won't always work.
		#   2. We need the "elbow out" ik solution but that isn't guaranteed yet.
		#TODO REMOVE JANKINESS - Jing-Chen
		self.tool = np.array([0,0,0])

		if self.left_limb.active:
			self.left_limb.setEETransformImpedance([leftUntuckedRotation.tolist(),leftUntuckedTranslation.tolist()], self.K, self.M, self.B)
		if self.right_limb.active:
			self.right_limb.setEETransformImpedance([rightUntuckedRotation.tolist(),rightUntuckedTranslation.tolist()], self.K, self.M, self.B)


	def UIStateLogic(self):
		if(type(self.UI_state)!= int):
			if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
				print("Robot Home")
				self.setRobotToDefault()
				self.left_limb.teleoperationState = 1
				self.right_limb.teleoperationState = 1
			if self.controller_mode == ControllerMode.ABSOLUTE:
				if (self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True and self.left_limb.teleoperationState == 1):
					print('\n\n\n\n resetting UI initial state \n\n\n\n\n')
					self.init_UI_state = self.UI_state
					self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])

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
							
							limb.init_pos = limb.sensedEETransform()

							print("BEGIN CLUTCHING, ZERO!")

							limb.teleoperationState = 2

					elif limb.teleoperationState == 2 or limb.teleoperationState == 3:
						limb.teleoperationState = 3
						#limb.setEEVelocity([0,0,0,0,0,0], tool = self.tool.tolist())
						limb.setEETransformImpedance(limb.sensedEETransform(), self.K, self.M, [[4*x for x in a] for a in self.B])

			if(self.base_active):
				self.baseControl()

			self.control('impedance')
	def baseControl(self):
		'''controlling base movement'''
		base_velocity = [0.5*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.5*(-self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][0])]
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
			self.controlArm(self.left_limb, mode)
			self.temp_robot_telemetry['rightArm'] = self.robot.sensedRightLimbPosition()

		if (self.mode == 'Physical') and self.left_limb.gripper_active: # and self.left_limb.teleoperationState == 2:
			closed_value = self.UI_state["controllerButtonState"]["leftController"]["squeeze"][0]
			if(closed_value > 0):
				self.robot.closeLeftRobotiqGripper()
			else:
				self.robot.openLeftRobotiqGripper()

		self.robot.addRobotTelemetry(self.temp_robot_telemetry)

	"""
	Control an arm based on UI state and other jazz. Might lock it.
	
	limb: Arm object.
	mode: One of position, velocity, or impedance.
	"""
	def controlArm(self, limb, mode):
		assert (mode in ['position', 'velocity', 'impedance']), "Invalid mode"
		if self.UI_state["controllerButtonState"][limb.joystick]["squeeze"][1] > 0.5:
			print("DRIVE LIMB " + limb.side)

			RR_final, RT_final, curr_transform = self.getTargetEETransform(limb)
			print("TARGET TRANSFORM:")
			print(RR_final)
			print(RT_final)
			print("CURRENT TRANSFORM:")
			print(curr_transform)
			print("RAW: ")
			print(np.array(self.UI_state["controllerPositionState"]
				[limb.joystick]["controllerPosition"]))
			print("HOMED:")
			print(np.array(self.init_UI_state["controllerPositionState"]
				[limb.joystick]["controllerPosition"]))

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
				limb.setEETransformImpedance(target_transform, self.K, self.M, self.B)

            #TODO remove jankness - Jing-Chen is this even doing anything??
			err = self.tool_target - self.tool
			self.tool = np.add(self.tool, 0.1 * err, out=self.tool, casting="unsafe")
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
		print('\n\n Translation Transforms \n')
		print(RT_final,RT_rw_rh)
		print('\n\n')
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
		return RR_final, RT_final, curr_transform

	def logTeleoperation(self,name):
		q = self.robotPoser.get()
		left_limb_command = q[10:16]
		right_limb_command = q[35:41]
		self.fileName = 'Teleoperation_log/motion' + ''.join(name) + '.csv'
		self.saveStartTime =  datetime.datetime.utcnow()
		self.saveEndTime = datetime.datetime.utcnow() + datetime.timedelta(0,3.1)
		fields = ['timestep', 'Left Shoulder', 'Left UpperArm', 'Left ForeArm', 'Left Wrist1','Left Wrist2','Left Wrist3','Right Shoulder', 'Right UpperArm', 'Right ForeArm', 'Right Wrist1','Right Wrist2','Right Wrist3','Left EE Transform', 'Right EE Transform' ]
		with open(self.fileName, 'w') as csvfile:
			# creating a csv writer object
			csvwriter = csv.writer(csvfile)
			csvwriter.writerow(fields)
		self.robot.setLeftLimbPositionLinear(left_limb_command,3)
		self.robot.setRightLimbPositionLinear(right_limb_command,3)

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

		print('Treated initial matrix - here is the transform')
		print(rotation_final.as_dcm())

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
