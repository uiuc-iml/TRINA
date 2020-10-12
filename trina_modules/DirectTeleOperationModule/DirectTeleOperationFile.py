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
from klampt.math import so3, so2

robot_ip = 'http://localhost:8080'


ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

roomname = "The Lobby"
zonename = "BasicExamples"
userId=0
roomId=-1
is_closed=0

class DirectTeleOperation:
	def __init__(self,Jarvis = None, debugging = False, mode = 'Kinematic'):
		self.mode = mode
		self.status = 'idle' #states are " idle, active"
		self.state = 'idle' #states are " idle, active"
		self.init_UI_state = {}
		self.dt = 0.025
		self.infoLoop_rate = 0.05
		# TODO: units? I (Patrick) think it's m/s
		self.max_arm_speed = 0.5
		self.robot = Jarvis
		self.components =  ['base','left_limb','right_limb','left_gripper']
		#self.robot.getComponents()
		self.left_limb_active = ('left_limb' in self.components)
		self.right_limb_active = ('right_limb' in self.components)
		self.base_active = ('base' in self.components)
		self.left_gripper_active = ('left_gripper' in self.components)
		self.right_gripper_active = ('right_gripper' in self.components)
		self.torso_active = ('torso' in self.components)
		self.temp_robot_telemetry = {'leftArm':[0,0,0,0,0,0],'rightArm':[0,0,0,0,0,0]}
		time.sleep(5)
		self.UI_state = {}
		self.init_pos_left = {}
		self.cur_pos_left = {}
		self.init_pos_right = {}
		self.init_headset_orientation = {}
		self.cur_pos_right = {}
		self.startup = True
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl+c)

		stateRecieverThread = threading.Thread(target=self._serveStateReceiver)
		main_thread = threading.Thread(target = self._infoLoop)
		stateRecieverThread.start()
		main_thread.start()

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
					# self.status = 'idle'
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
			if(self.left_limb_active):
				self.init_pos_left = self.robot.sensedLeftEETransform()
			if(self.right_limb_active):
				self.init_pos_right = self.robot.sensedRightEETransform()
			self.init_headset_orientation = self.treat_headset_orientation(self.init_UI_state['headSetPositionState']['deviceRotation'])
		while(True):
			if self.state == 'idle':
				# print("_serveStateReceiver: idling")
				pass
			elif self.state == 'active':
				# print('_serveStateReceiver:active')
				self.last_time = time.time()
				if(self.left_limb_active):
					self.cur_pos_left = self.robot.sensedLeftEETransform()
				if(self.right_limb_active):
					self.cur_pos_right = self.robot.sensedRightEETransform()
				self.UI_state = self.robot.getUIState()
				self.UIStateLogic()
				time.sleep(self.dt)


	def setRobotToDefault(self):
		leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
		rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
		rightUntuckedRotation = np.array([
									0.9996677819374474, -0.020138967102307673, 0.016085638324823164,
									-0.025232828744241535, -0.6373999040791976, 0.7701199040626047,
									-0.005256435087453611, -0.7702699424772351, -0.6376963114259705
								])
		rightUntuckedTranslation = np.array([0.6410086795413383, -0.196298410887376, 0.8540173127153597])
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
		if('left_limb' in self.components):
			#self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,2)
			self.robot.setLeftEEInertialTransform([leftUntuckedRotation.tolist(),leftUntuckedTranslation.tolist()],2)
		if('right_limb' in self.components):
			#self.robot.setRightLimbPositionLinear(rightUntuckedConfig,2)
			self.robot.setRightEEInertialTransform([rightUntuckedRotation.tolist(),rightUntuckedTranslation.tolist()],2)
			pass


	def UIStateLogic(self):
		if(type(self.UI_state)!= int):
			if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
				self.setRobotToDefault()
			if (self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True):
				print('\n\n\n\n resetting UI initial state \n\n\n\n\n')
				self.init_UI_state = self.UI_state
				self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])
				self.init_pos_right = self.robot.sensedRightEETransform()
				self.init_pos_left = self.robot.sensedLeftEETransform()

			if(self.base_active):
				self.baseControl()
			# TODO: Switch this to self.velocityControl
			self.positionControl()


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

	def positionControl(self):
		'''controlling arm movement with position command
		'''
		if(self.left_limb_active):
			self.positionControlArm('left')
			self.temp_robot_telemetry['leftArm'] = self.robot.sensedLeftLimbPosition()
		if(self.right_limb_active):
			self.positionControlArm('right')
			self.temp_robot_telemetry['rightArm'] = self.robot.sensedRightLimbPosition()
		self.robot.addRobotTelemetry(self.temp_robot_telemetry)

		left_pos = self.robot.sensedLeftEETransform()
		right_pos = self.robot.sensedRightEETransform()
		print('left arm pos:')
		print(left_pos)
		print('right arm pos:')
		print(right_pos)
		print('\n\n\n\n\n\n\n')
		self.setRobotToDefault()


	def positionControlArm(self,side):
		actual_dt = self.dt
		assert (side in ['left','right']), "invalid arm selection"
		joystick = side+"Controller"
		if self.UI_state["controllerButtonState"][joystick]["squeeze"][1] > 0.5 :
			RR_final, RT_final, _ = self.getEETransform(side)
			start_time = time.time()
			if(side == 'right'):
				print('\n\n\n\n\n\n moving right arm')
				print('From:')
				print(RR_final)
				print(RT_final)
				print('\n\n\n\n\n\n\n')
				self.robot.setRightEEInertialTransform([RR_final,RT_final],actual_dt)
			else:
				print('\n\n\n\n\n\n moving left arm')
				print('From:')
				print(RR_final)
				print(RT_final)
				print('\n\n\n\n\n\n\n')
				self.robot.setLeftEEInertialTransform([RR_final,RT_final],actual_dt)
				if((self.mode == 'Physical') and self.left_gripper_active):
					closed_value = self.UI_state["controllerButtonState"]["leftController"]["squeeze"][0]*2.3
					if(closed_value >= 0.2):
						self.robot.setLeftGripperPosition([closed_value,closed_value,closed_value,0])
					else:
						self.robot.setLeftGripperPosition([0,0,0,0])

	def velocityControl(self):
		'''controlling arm movement with velocity command
		'''
		self.velocityControlArm('left')
		self.velocityControlArm('right')

	def velocityControlArm(self,side):
		assert (side in ['left','right']), "invalid arm selection"
		joystick = side+"Controller"
		if self.UI_state["controllerButtonState"][joystick]["squeeze"][1] > 0.5:
			RR_final, RT_final, curr_transform = self.getEETransform(self, side)
			#TODO STARTUP: Make this part work
			vel = (np.array(RT_final)-np.array(curr_position))/self.dt
			vel_norm = np.linalg.norm(vel)
			max_vel = 0.5
			actual_dt = self.dt
			if(vel_norm > max_vel):
				vel = (vel*max_vel/vel_norm).tolist()
			else:
				vel = vel.tolist()

			start_time = time.time()

			# we now calculate the difference between current and desired positions:
			Delta_RR = (curr_orientation.inv()*RR_final).inv().as_rotvec()
			rotation_norm = np.linalg.norm(Delta_RR)
			max_rot = 1
			if(rotation_norm > max_rot):
				Delta_RR = (Delta_RR*max_rot/rotation_norm).tolist()
			else:
				Delta_RR = Delta_RR.tolist()
			if(side == 'right'):
				self.robot.setRightEEVelocity(vel+Delta_RR, tool = [0,0,0])
			else:
				print('moving left arm \n\n\n',vel+Delta_RR)
				self.robot.setLeftEEVelocity(vel+Delta_RR, tool = [0,0,0])
				if((self.mode == 'Physical') and self.left_gripper_active):
					closed_value = self.UI_state["controllerButtonState"]["leftController"]["squeeze"][0]*2.3
					if(closed_value >= 0.2):
						self.robot.setLeftGripperPosition([closed_value,closed_value,closed_value,0])
					else:
						self.robot.setLeftGripperPosition([0,0,0,0])

	def getEETransform(self, side):
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
		side: str: ['left', 'right']
			Which arm to compute transform for.

		Returns
		----------------
		tuple: (np.ndarray, np.ndarray, tuple)
			Desired rotation transform and translation transform for the
			requested arm, (R,t) tuple representing the current transform of
			the requested arm
		"""
		joystick = side+"Controller"
		R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
		if(side == 'right'):
			[RR_rw_rh,RT_rw_rh] = self.init_pos_right
			curr_transform = self.robot.sensedRightEETransform()
		elif(side == 'left'):
			[RR_rw_rh,RT_rw_rh] = self.init_pos_left
			curr_transform = self.robot.sensedLeftEETransform()
		RT_rw_rh = np.array(RT_rw_rh)
		RT_cw_cc = np.array(self.UI_state["controllerPositionState"]
			[joystick]["controllerPosition"])
		RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"]
			[joystick]["controllerPosition"])
#		RT_final = ( RT_rw_rh
#			+ (self.init_headset_orientation.as_dcm() @ R_cw_rw
#			@ (RT_cw_cc - RT_cw_ch).T) ).tolist()
		RT_final = ( RT_rw_rh
			+ (np.matmul(np.matmul(self.init_headset_orientation.as_dcm(), R_cw_rw), 
			(RT_cw_cc - RT_cw_ch).T)) ).tolist()
		print('\n\n Translation Transforms \n')
		print(RT_final,RT_rw_rh)
		print('\n\n')
		RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))

		init_quat = np.array(
			self.init_UI_state["controllerPositionState"]
				[joystick]['controllerRotation'])

		R_cw_rw = R.from_dcm(R_cw_rw)

		# world_adjustment_matrix = R_cw_rw
		world_adjustment_matrix = R.from_dcm(np.eye(3))
		init_angle = (np.pi/180)*init_quat[3]

		right_handed_init_vec = np.array(
			[-init_quat[2],init_quat[0],-init_quat[1]]) * (-init_angle)

		#transform it to right handed:
		curr_quat = np.array(self.UI_state["controllerPositionState"][joystick]
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
