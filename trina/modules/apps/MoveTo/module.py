import time,math
import threading
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
from Settings import trina_settings
from Jarvis import JarvisModuleBase

class MoveToModule(JarvisModuleBase):
	def __init__(self,Jarvis = None, debugging = False):
		JarvisModuleBase.__init__(self,Jarvis)
		self.robot = self.jarvis.robot
		self.status = 'idle' #states are " idle, active"

		self.components = self.robot.activeComponents()
		#self.robot.getComponents()
		self.left_limb_active = ('left_limb' in self.components)
		self.right_limb_active = ('right_limb' in self.components)
		self.base_active = ('base' in self.components)
		self.left_gripper_active = ('left_gripper' in self.components)
		self.right_gripper_active = ('right_gripper' in self.components)
		self.torso_active = ('torso' in self.components)
		self.init_UI_state = {}
		self.temp_robot_telemetry = {'leftArm':[0,0,0,0,0,0],'rightArm':[0,0,0,0,0,0]}
		self.UI_state = {}
		self.init_pos_left = {}
		self.cur_pos_left = {}
		self.init_pos_right = {}
		self.init_headset_orientation = {}
		self.cur_pos_right = {}
		self.user_saved_configs = {}
		self.startup = True

		self.startSimpleThread(self._stateReceiverLoop,rate=0.025,name='stateReceiver')
		self.startSimpleThread(self._infoLoop,rate=0.05,name='infoLoop')

	def _infoLoop(self):
		self.jarvis.log_health()
		status = self.jarvis.getActivityStatus()

		if status == 'active':
			if self.status == 'idle':
				print('\n\n\n\n starting up MoveTo Module! \n\n\n\n\n')
				self.status = 'active'

		elif status == 'idle':
			if self.status == 'active':
				self.status = 'idle'

	def _serveStateReciever(self):
		if self.status == 'idle':
			# print("_serveStateReciever: idling")
			self.startup = True
		elif self.status == 'active':
			if self.startup:
				#wait for the UI state to start up
				self.init_UI_state = self.jarvis.ui.getUIState()
				if self.init_UI_state != 0:
					print('started the initial values for the variables')
					self.startup = False
					if self.left_limb_active:
						self.init_pos_left = self.robot.sensedLeftEETransform()
					if self.right_limb_active:
						self.init_pos_right = self.robot.sensedRightEETransform()
					self.init_headset_orientation = self.treat_headset_orientation(self.init_UI_state['headSetPositionState']['deviceRotation'])
					# self.setRobotToDefault()
			else:
				if self.left_limb_active:
					self.cur_pos_left = self.robot.sensedLeftEETransform()
				if self.right_limb_active:
					self.cur_pos_right = self.robot.sensedRightEETransform()
				self.UI_state = self.robot.getUIState()
				self.MoveToLogic()
				self.UIStateLogic()

	def setRobotToDefault(self):
		leftUntuckedConfig = trina_settings.left_arm_config('untucked')
		rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
		if 'left_limb' in self.components:
			self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,2)
		if 'right_limb' in self.components:
			self.robot.setRightLimbPositionLinear(rightUntuckedConfig,2)

	def UIStateLogic(self):
		if isinstance(self.UI_state,int):
			print("Looks like UI is disconnected?")
			self.startup = True
			return
		if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
			self.setRobotToDefault()
		if self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True:
			print('\n\n\n\n resetting UI initial state \n\n\n\n\n')
			self.init_UI_state = self.UI_state
			self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])

		if self.base_active:
			self.baseControl()
		self.positionControl()


	def baseControl(self):
		'''controlling base movement'''
		base_velocity = [0.5*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.5*(-self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][0])]

		try:
			self.robot.setBaseVelocity(base_velocity)
		except:
			print("setBaseVelocity not successful")
			pass

	def positionControl(self):
		'''controlling arm movement with position command
		variable naming:
			LT_cw_cc => Left Traslational matrix from Controller World to Controller Current
			RR_rw_rh => Right Rotational matrix from Robot World to Robot Home
			RR_rw_rh_T => Right Rotational matrix from Robot World to Robot Home Transpose
			R_cw_rw  => Rotational matrix from Controller World to Robot World

			cc------controller current
			rc------robot current

			cw------controller world
			rw------robot world

			ch------controller homw
			rh------robot home
		'''
		if(self.left_limb_active):
			self.positionControlArm('left')
			self.temp_robot_telemetry['leftArm'] = self.robot.sensedLeftLimbPosition()
		if(self.right_limb_active):
			self.positionControlArm('right')
			self.temp_robot_telemetry['rightArm'] = self.robot.sensedRightLimbPosition()
		self.ui.addRobotTelemetry(self.temp_robot_telemetry)

	def positionControlArm(self,side):
		actual_dt = self.dt
		assert (side in ['left','right']), "invalid arm selection"
		R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
		joystick = side+"Controller"
		if self.UI_state["controllerButtonState"][joystick]["squeeze"][1] > 0.5 :
			if(side == 'right'):
				[RR_rw_rh,RT_rw_rh] = self.init_pos_right
				curr_position = np.array(self.robot.sensedRightEETransform()[1])

			elif(side == 'left'):
				[RR_rw_rh,RT_rw_rh] = self.init_pos_left
				curr_position = np.array(self.robot.sensedLeftEETransform()[1])
			RT_rw_rh = np.array(RT_rw_rh)
			RT_cw_cc = np.array(self.UI_state["controllerPositionState"][joystick]["controllerPosition"])
			RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"][joystick]["controllerPosition"])
			RT_final = np.add(RT_rw_rh, np.matmul(np.matmul(self.init_headset_orientation.as_matrix(),R_cw_rw), np.subtract(RT_cw_cc,RT_cw_ch).transpose())).tolist()

			RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))

			init_quat = np.array(self.init_UI_state["controllerPositionState"][joystick]['controllerRotation'])

			R_cw_rw = R.from_dcm(R_cw_rw)

			# world_adjustment_matrix = R_cw_rw
			world_adjustment_matrix = R.from_dcm(np.eye(3))
			init_angle = (np.pi/180)*init_quat[3]

			right_handed_init_vec = np.array([-init_quat[2],init_quat[0],-init_quat[1]])*(-init_angle)
			#transform it to right handed:

			curr_quat = np.array(self.UI_state["controllerPositionState"][joystick]['controllerRotation'])
			curr_angle = (np.pi/180)*curr_quat[3]
			right_handed_curr_vec = np.array([-curr_quat[2],curr_quat[0],-curr_quat[1]])*(-curr_angle)

			RR_cw_ch = R.from_rotvec(right_handed_init_vec)
			RR_cw_ch_T = RR_cw_ch.inv()
			RR_cw_cc = R.from_rotvec(right_handed_curr_vec)
			RR_ch_cc = self.init_headset_orientation*(RR_cw_ch_T*RR_cw_cc)*self.init_headset_orientation.inv()

			RR_final = (RR_rw_rh*RR_ch_cc).as_matrix().flatten().tolist()

			start_time = time.process_time()
			if(side == 'right'):
				print('\n\n\n\n\n\n moving right arm \n\n\n\n\n\n\n')

				self.robot.setRightEEInertialTransform([RR_final,RT_final],actual_dt)

			else:
				print('\n\n\n\n\n\n moving left arm \n\n\n\n\n\n\n')

				self.robot.setLeftEEInertialTransform([RR_final,RT_final],actual_dt)
				if((self.mode == 'Physical') and self.left_gripper_active):
					closed_value = self.UI_state["controllerButtonState"]["leftController"]["squeeze"][0]*2.3
					if(closed_value >= 0.2):
						self.robot.setLeftGripperPosition([closed_value,closed_value,closed_value,0])
					else:
						self.robot.setLeftGripperPosition([0,0,0,0])

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
		print(rotation_final.as_dcm())
		return rotation_final

	def MoveToLogic(self):
		save_flag,save_name = self.jarvis.ui.getSaveConfigSignalUI()
		load_flag,load_name = self.jarvis.ui.getLoadConfigSignalUI()
		if save_flag:
			self.user_saved_configs[save_name] = self.temp_robot_telemetry
		
		if load_flag:
			self.jarvis.setLeftLimbPosition(self.user_saved_configs[load_name]['leftArm'])
			self.jarvis.setRightLimbPosition(self.user_saved_configs[load_name]['rightArm'])





if __name__ == "__main__" :
	my_controller = MoveTo()
	while True:
		time.sleep(1.0)