import time,math,datetime
import threading
from motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import numpy as np
from scipy.spatial.transform import Rotation as R
import os,csv,sys
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback
from multiprocessing import Pool, TimeoutError
import uuid
from klampt import io
# from Modules import *
import trina_modules
import sys, inspect
# import command_server

class Jarvis:

	def __init__(self):
		self.interface = RedisInterface(host="localhost")
		self.interface.initialize()
		self.server = KeyValueStore(self.interface)
		# should not instantiate commanserver
		# self.command_server = CommandServer()

	def sensedBaseVelocity(self):
		return self.server["ROBOT_STATE"]["Velocity"]["Base"].read()

	def sensedLeftLimbVelocity(self):
		return self.server["ROBOT_STATE"]["Velocity"]["LeftArm"].read()

	def sensedRightLimbVelocity(self):
		return self.server["ROBOT_STATE"]["Velocity"]["RightArm"].read()

	def sensedBasePosition(self):
		return self.server["ROBOT_STATE"]["Position"]["Base"].read()

	def sensedTorsoPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["Torso"].read()

	def sensedLeftLimbPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["LeftArm"].read()

	def sensedRightLimbPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["RightArm"].read()

	def sensedBasePosition(self):
		return self.server["ROBOT_STATE"]["Position"]["Base"].read()

	def sensedLeftGripperPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["LeftGripper"].read()

	def setLeftLimbPosition(self,q):
		command = send_command(Motion.setLeftLimbPosition,q)
		current_list = server['ROBOT_COMMAND']['3'].read()
		server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setRightLimbPosition(self,q):
		command = send_command(Motion.setRightLimbPosition,q)
		current_list = server['ROBOT_COMMAND']['3'].read()
		server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setLeftLimbPositionLinear(self,q,duration):
		command = send_command(Motion.setLeftLimbPositionLinear,q,duration)
		current_list = server['ROBOT_COMMAND']['3'].read()
		server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setRightLimbPositionLinear(self,q,duration):
		command = send_command(Motion.setRightLimbPositionLinear,q,duration)
		current_list = server['ROBOT_COMMAND']['3'].read()
		server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setVelocity(self,qdot):
        command = send_command(Motion.setVelocity,qdot)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setLeftLimbVelocity(self,qdot):
        command = send_command(Motion.setLeftLimbVelocity,qdot)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setRightLimbVelocity(self,qdot):
        command = send_command(Motion.setRightLimbVelocity,qdot)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setLeftEEInertialTransform(self,Ttarget,duration):
        command = send_command(Motion.setLeftEEInertialTransform,Ttarget,duration)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setLeftEEVelocity(self,v,tool):
		if not tool:
			tool = [0,0,0]
        command = send_command(Motion.setLeftEEVelocity,v,tool)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setRightEEInertialTransform(self,Ttarget,duration):
		command = send_command(Motion.setRightEEInertialTransform,Ttarget,duration)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)


	def setRightEEVelocity(self, v ,tool):
		if not tool:
			tool = [0,0,0]
        command = send_command(Motion.setRightEEVelocity,v,tool)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	# def sensedLeftEETransform(self):
	# 	"""Return the transform w.r.t. the base frame"""
	# 	return self.s.sensedLeftEETransform()
    #
	# def sensedRightEETransform(self):
	# 	"""Return the transform w.r.t. the base frame"""
	# 	return self.s.sensedRightEETransform()

	def setBaseTargetPosition(self, q, vel):
        command = send_command(Motion.setBaseTargetPosition,q,vel)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setBaseVelocity(self, q):
        command = send_command(Motion.setBaseVelocity,q)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setTorsoTargetPosition(self, q):
		command = send_command(Motion.setTorsoTargetPosition,q)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setLeftGripperPosition(self, position):
		command = send_command(Motion.setLeftGripperPosition,position)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	def setLeftGripperVelocity(self,velocity):
		command = send_command(Motion.setLeftGripperVelocity,velocity)
        current_list = server['ROBOT_COMMAND']['3'].read()
        server['ROBOT_COMMAND']['3'] = current_list.append(command)

	# def getKlamptCommandedPosition(self):
	# 	return self.s.getKlamptCommandedPosition()
    #
	# def getKlamptSensedPosition(self):
	# 	return self.s.getKlamptSensedPosition()

	def shutdown(self):
        command = send_command(Motion.shutdown)
        current_list = server['ROBOT_COMMAND']['0'].read()
        server['ROBOT_COMMAND']['0'] = current_list.append(command)

	# def isStarted(self):
	# 	return self.s.isStarted()
    #
	# def isShutDown(self):
	# 	return self.s.isShutDown()

	# def moving(self):
	# 	"""Returns true if the robot is currently moving."""
	# 	return self.s.moving()

	# def mode(self):
	# 	return self.s.mode()

	# def stopMotion(self):
	# 	self.s.stopMotion()
    #
	# def resumeMotion(self):
	# 	self.s.resumeMotion()
    #
	# def mirror_arm_config(self,config):
	# 	return self.s.mirror_arm_config(config)

	# def getWorld(self):
	# 	return self.world
    #
	# def cartesianDriveFail(self):
	# 	return self.s.cartesianDriveFail()
    #
	# def sensedLeftEEVelocity(self,local_pt = [0,0,0]):
	# 	return self.s.sensedLeftEEVelcocity(local_pt)
    #
	# def sensedRightEEVelocity(self,local_pt = [0,0,0]):
	# 	return self.s.sensedRightEEVelcocity(local_pt)
################################## All Mighty divider between motion and UI###############################

	def getRayClickUI(self):
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
		id = '$'+ uuid.uuid1().hex
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':False, 'MSG':''}
		# ask the user to click on a destination in the map, returns 2 rays in reem
		self._do_rpc({'funcName':'getRayClick','args':{'id':str(id)}})
		reply = self.checkFeedback(id)
		return  reply

	def addTextUI(self, text, position):
		"""add text to specfified location on UI screen.

		args:
			text: (str) content you wish to add
			position: (str) positions of the text: (x,y) position of the upper left corner
			of the text on the screen

		return:
			nothing

		blocking?:
			no
		"""
		self._do_rpc({'funcName':'addText','args':{'text':text,'position':position}})
		return

	def addConfirmationUI(self,title,text):
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
		id = '$'+ uuid.uuid1().hex
		self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':False, 'MSG':''}
		self._do_rpc({'funcName':'addConfirmation','args':{'id':str(id),'title':title,'text':text}})
		reply = self.checkFeedback(id)
		return  reply

	def addPromptUI(self,title,text):
		id = '$'+ uuid.uuid1().hex
		# TODO
		return  id

	def addInputBoxUI(self,title,text,fields):
		id = '$'+ uuid.uuid1().hex
		# TODO
		return id

	def sendTrajectoryUI(self,trajectory,animate):
		"""send a trajectory to UI, UI will add the path preview and animate? the robot ghost immediately for only once
		args:
			trajectory: (klampt obj) the traj calculated
			animate: (bool) if user wants to animate the path

		return:
			nothing

		blocking?:
			no
		"""
		trajectory = io.loader.toJson(trajectory,'Trajectory')
		self._do_rpc({'funcName':'sendTrajectory','args':{'trajectory':trajectory, 'animate':animate}})
		return













	# helper func
	def send_command(command,*args):
		final_string = str(command)+ '('
		for index,arg in enumerate(args):
			if(index != len(args)-1):
				final_string += '{},'
			else:
				final_string += '{}'
		final_string = (final_string + ')')
		final_string = final_string.format(*args)
		return final_string

	def checkFeedback(self,id):
		while not self.server['UI_FEEDBACK'][str(id)]['REPLIED'].read():
			continue
		return self.server['UI_FEEDBACK'][str(id)]['MSG'].read()

	def _do_rpc(self,msg):
		commandQueue = self.server["UI_END_COMMAND"].read()
		commandQueue.append(msg)
		self.server["UI_END_COMMAND"] = commandQueue
		print("commandQueue", commandQueue)
		time.sleep(0.0001)

if __name__=="__main__":
	server = Jarvis()
