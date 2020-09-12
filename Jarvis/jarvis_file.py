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

		# should not instantiate commanserver
		# self.command_server = CommandServer()

	# def shutdown(self):
	#     self.command_server.shutdown()

	def setPosition(self, q):
		return 0

	def setLeftLimbPosition(self, q):
		command = self.send_command('self.robot.setLeftLimbPosition', str(q))

		return 0

	def setRightLimbPosition(self, q):
		command = self.send_command('self.robot.setRightLimbPosition', str(q))

		return 0

	def setLeftLimbPositionLinear(self, q, duration):
		command = self.send_command(
			'self.robot.setLeftLimbPositionLinear', str(q), str(duration))

		return 0

	def setRightLimbPositionLinear(self, q, duration):
		command = self.send_command(
			'self.robot.setRightLimbPositionLinear', str(q), str(duration))

		return 0

	def setVelocity(self, qdot):
		command = self.send_command('self.robot.setVelocity', str(qdot))

	def setLeftLimbVelocity(self, qdot):
		command = self.send_command('self.robot.setLeftLimbVelocity', str(qdot))

	def setRightLimbVelocity(self, qdot):
		command = self.send_command('self.robot.setRightLimbVelocity', str(qdot))

	def setLeftEEInertialTransform(self, Ttarget, duration):
		command = self.send_command(
			'self.robot.setLeftEEInertialTransform', str(Ttarget), str(duration))

	def setLeftEEVelocity(self, v, tool):
		if not tool:
			tool = [0, 0, 0]
		command = self.send_command(
			'self.robot.setLeftEEVelocity', str(v), str(tool))

	def setRightEEInertialTransform(self, Ttarget, duration):
		command = self.send_command(
			'self.robot.setRightEEInertialTransform', str(Ttarget), str(duration))

	def setRightEEVelocity(self, v, tool):
		if not tool:
			tool = [0, 0, 0]
		command = self.send_command(
			'self.robot.setRightEEVelocity', str(v), str(tool))

	def setBaseTargetPosition(self, q, vel):
		command = self.send_command(
			'self.robot.setBaseTargetPosition', str(q), str(vel))

	def setBaseVelocity(self, q):
		command = self.send_command('self.robot.setBaseVelocity', str(q))

	def setTorsoTargetPosition(self, q):
		command = self.send_command('self.robot.setTorsoTargetPosition', str(q))

	def setLeftGripperPosition(self, position):
		command = self.send_command(
			'self.robot.setLeftGripperPosition', str(position))

	def setLeftGripperVelocity(self, velocity):
		command = self.send_command(
			'self.robot.setLeftGripperVelocity', str(velocity))

	# def shutdown(self):
	# 	self.shut_down = True
	# 	self.s.shutdown()

	def isStarted(self):
		return self.server['ROBOT_INFO']['Started'].read()

	def isShutDown(self):
		return self.server['ROBOT_INFO']['Shutdown'].read()

	def moving(self):
		"""Returns true if the robot is currently moving."""
		return self.server['ROBOT_INFO']['Moving'].read()

	def mode(self):
		return self.server['ROBOT_INFO']['MODE'].read()

	def stopMotion(self):
		command = self.send_command('self.robot.stopMotion')
		# self.s.stopMotion()

	def resumeMotion(self):
		command = self.send_command('self.robot.resumeMotion')

	# def mirror_arm_config(self,config):
	#     command = self.send_command('self.robot.resumeMotion', str(q))
	#     queue = self.server['ROBOT_COMMAND'][self.name].read()
	#     queue.append(command)
	#     self.server['ROBOT_COMMAND'][self.name] = queue
		# return self.s.mirror_arm_config(config)

	def getWorld(self):
		return self.server["WORLD"].read()
		# return self.world

	def cartesianDriveFail(self):
		return self.server["ROBOT_INFO"]["CartesianDrive"].read()
		# return self.s.cartesianDriveFail()

	def sensedLeftEEVelocity(self, local_pt=[0, 0, 0]):
		return self.server["ROBOT_STATE"]["VelocityEE"]["LeftArm"].read()
		# return self.s.sensedLeftEEVelcocity(local_pt)

	def sensedRightEEVelocity(self, local_pt=[0, 0, 0]):
		return self.server["ROBOT_STATE"]["VelocityEE"]["RightArm"].read()
		# return self.s.sensedRightEEVelcocity(local_pt)

	def getUIState(self):
		return self.server["UI_STATE"].read()

	def getRobotState(self):
		return self.server["ROBOT_STATE"].read()

	def getComponents(self):
		return self.server["ROBOT_INFO"]["Components"].read()

	def addRobotTelemetry(self, value):
		self.server["robotTelemetry"] = value

	def getKlamptSensedPosition(self):
		return self.server["ROBOT_STATE"]["KlamptSensedPos"].read()
		# return self.s.getKlamptSensedPosition()

	def getKlamptCommandedPosition(self):
		return self.server["ROBOT_STATE"]["KlamptCommandPos"].read()
		# return self.s.getKlamptCommandedPosition()

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

	def sensedLeftEETransform(self):
		return self.server["ROBOT_STATE"]["PositionEE"]["LeftArm"].read()

	def sensedRightEETransform(self):
		return self.server["ROBOT_STATE"]["PositionEE"]["RightArm"].read()

	def sensedGripperPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["LeftGripper"].read()

	def sensedRobotq(self):
		return self.server["ROBOT_STATE"]["Position"]["Robotq"].read()

	def sensedRightLimbPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["RightArm"].read()

	def sensedLeftLimbPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["LeftArm"].read()

	def getActivityStatus(self):
		return self.server['ACTIVITY_STATUS'][self.name].read()

	def get_point_clouds(self):
		return self.sensor_module.get_point_clouds()

	def get_rgbd_images(self):
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

	def log_health(self, status=True):
		self.server["HEALTH_LOG"][self.name] = [status, time.time()]

	def changeActivityStatus(self, to_activate, to_deactivate=[]):
		command = self.send_command(
			'self.switch_module_activity', str(to_activate), str(to_deactivate))

	def getTrinaTime(self):

		return self.server['TRINA_TIME'].read()

##############################################################3 All Mighty divider between motion and UI###############################
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

	# helper func
	def send_command(self,command,*args):
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
