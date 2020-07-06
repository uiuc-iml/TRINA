import os,sys
import signal
import time
import math
from threading import Thread, Lock, RLock
import threading
import numpy as np
from multiprocessing import Process, Pipe
import logging
from copy import deepcopy,copy
from klampt.math import vectorops,so3
from klampt.model import ik, collide
from klampt import WorldModel, vis
from TRINAConfig import *
import sensor_msgs
from motion import Motion
from matplotlib import pyplot as plt

class ForceControl:
	def __init__(self,Jarvis = None, debugging = False, mode = 'Kinematic'):
		#if true, run a test locally, otherwise, communicate with Jarvis to get state
		self.debugging = debugging
		self.state = 'idle' #states are " idle, active"
		self.infoLoop_rate = 0.02
		self.exit_flag = False
		self.mode = mode
		self.status = 'idle'
		time.sleep(10)
		if self.debugging:
			pass
		else:
			self.jarvis = Jarvis


		self.kp = None
		self.kd = None
		self.ki = None
		self.max_velocity = None
		self.min_velocity = None

		self.dt


		main_thread = threading.Thread(target = self._mainLoop)
		state_thread = threading.Thread(target = self._infoLoop)
		state_thread.start()
		main_thread.start()

	def return_threads(self):
		return [self._infoLoop,self._mainloop]

	def _infoLoop(self):
		if self.debugging:
			pass

		while not self.exit_flag:
			self.jarvis.log_health()
			loop_start_time = time.time()
			#check if there is new information
			if self.debugging:
				pass
			else:
				status = self.jarvis.getActivityStatus()
				if(status == 'active'):
					if(self.status == 'idle'):
						print('\n\n\n\n starting up Autonomous Navigation Module! \n\n\n\n\n')
						self.activate()
				elif(status == 'idle'):
					if self.status == 'active':
						self.deactivate()

			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.infoLoop_rate:
				time.sleep(self.infoLoop_rate)
			else:
				time.sleep(0.001) #sleep a small amount of time to avoid occupying the lock
		print('----------')
		print('ForceControl: state thread exited')

	#Here we assume that 
	#There are a few different input formats to the force control algorithm:
	# - The most basic version, a function f(t) that gives T,Y,v_des,f_des
	# - Another version, where a desired EE trajectory and f trajectory is given, along with T,Y that are not time varying


	def _mainLoop(self):
		while not self.exit_flag:
			loop_start_time = time.time()
			self.last_timestamp = time.time()
			if self.state == 'idle':							
				pass
			elif self.state == 'active':
				#self.current_time = 0.0
				#self.state = 'executing'
				pass

			if self.state == 'executing':
				if self.input_mode == 'debugging':
					self.total_time = 10.0

				if self.current_time <= self.total_time:
					if self.input_mode == 'debugging':
						v_des = ForceControl.getVelocity(current_time,current_task_transform)
						w_des = ForceControl.getWrench(current_time)
						T,Y = ForceControl.getSelectionMatrices()
						kp = 
						ki = 
						kd = 
						max_velocity = np.array([0.05,0.05,0.05]) #need to have the correct dimension
						min_velocity = np.array([0.05,0.05,0.05]) #need to have the correct dimension
						task_transform = ForceControl.getTaskFrameTransform()
						
					elif self.input_mode == 'trajectory':
						self._sharedLock.acquire()
						#TODO calculate v_des from trajectory
						v_des = copy(self.v_des)
						w_des = copy(self.w_des)

				#check validity
				if np.linalg.norm(T.T@Y) > 1e-10:
					raise RuntimeError('Wrong selection matrices given')


				#get the current EE transform
				current_EE_transform = self.Jarvis.sensedLeftEETransform()
				current_task_transform = so3.mul(current_EE_transform,task_transform)
				current_raw_wrench = self.Jarvis.sensedLeftEEWrench(frame = 'local') 
				current_wrench = ForceControl.EEtoTask(current_raw_wrench,task_transform) #get wrench in the task frame

				self.last_error = copy(wrench_error)
				self.wrench_error = np.array(w_des) - Y@np.array(current_wrench)
				d_error = (wrench_error-last_error)/dt
				self.accumulated_error = accumulated_error + wrench_error

				v_des_from_wrench = np.dot(kp,wrench_error) + np.dot(kd,d_error) + np.dot(ki,accumulated_error)
				#clip the velocities for safety
				v_des_total = np.clip(v_des_total,min_velocity,max_velocity)
				v_des_total = T.T@np.array(v_des) + Y.T@v_des_from_wrench
				
				self.Jarvis.setLeftEEVelocity(v_des_total.tolist(), tool = task_transform[1])
				#add things to handle the case where cartesianDrive failed
				if self.Jarvis.cartesianDriveFail():
					print("singular configuration encountered")
					break

			self.current_time += self.dt
			elapsed_time = time.time() - loop_start_time
			if elapsed_time < dt:
				time.sleep(-elapsed_time + dt)
			else:
				time.sleep(0.0001)

	def _shutdown(self):
		self._sharedLock.acquire()
		self.exit_flag = True
		self._sharedLock.release()
		if self.visualization:
			vis.kill()
		return


	# def debugMain(self,v_des,w_des,T,Y,kp,ki,kd,max_velocity,min_velocity,task_transform):
	# 	"""
	# 	Right now, we assume it is the left EE only for debugging
	# 	"""
	# 	#T,Y = ForceControl.getSelectionMatrices()
		
	# 	#check validity
	# 	if np.linalg.norm(T.T@Y) > 1e-10:
	# 		raise RuntimeError('Wrong selection matrices given')
	# 	(_,size_k) = np.shape(T)
	# 	kp = 
	# 	ki = 
	# 	kd = 
	# 	max_velocity = np.array([0.05,0.05,0.05]) #need to have the correct dimension
	# 	min_velocity = np.array([0.05,0.05,0.05]) #need to have the correct dimension
	# 	task_transform = ForceControl.getTaskFrameTransform()
	# 	dt = 0.01
	# 	current_time = 0.0
	# 	total_time = 10
	# 	wrench_error = np.zeros(size_k)
	# 	last_error = np.zeros(size_k)
	# 	accumulated_error = np.zeros(size_k)
	# 	while current_time <= total_time:	
	# 		loop_start_time = time.time()
	# 		#get the current EE transform
	# 		current_EE_transform = self.Jarvis.sensedLeftEETransform()
	# 		current_task_transform = so3.mul(current_EE_transform,task_transform)
	# 		v_des = ForceControl.getVelocity(current_time,current_task_transform)
	# 		w_des = ForceControl.getWrench(current_time)

	# 		current_raw_wrench = self.Jarvis.sensedLeftEEWrench(frame = 'local') 
	# 		current_wrench = ForceControl.EEtoTask(current_raw_wrench,task_transform) #get wrench in the task frame

	# 		last_error = copy(wrench_error)
	# 		wrench_error = np.array(w_des) - Y@np.array(current_wrench)
	# 		d_error = (wrench_error-last_error)/dt
	# 		accumulated_error = accumulated_error + wrench_error

	# 		v_des_from_wrench = np.dot(kp,wrench_error) + np.dot(kd,d_error) + np.dot(ki,accumulated_error)
	# 		#clip the velocities for safety
	# 		v_des_total = np.clip(v_des_total,min_velocity,max_velocity)

	# 		v_des_total = T.T@np.array(v_des) + Y.T@v_des_from_wrench
			
	# 		self.Jarvis.setLeftEEVelocity(v_des_total.tolist(), tool = task_transform[1])
	# 		#add things to handle the case where cartesianDrive failed
	# 		if self.Jarvis.cartesianDriveFail():
	# 			print("singular configuration encountered")
	# 			break


	# 		current_time += dt
	# 		elapsed_time = time.time() - loop_start_time
	# 		if elapsed_time < dt:
	# 			time.sleep(-elapsed_time + dt)
	# 		else:
	# 			time.sleep(0.0001)

	
		
	@classmethod
	def EEtoTask(cls,wrench,T_task):
		"""
		Parameters:
		-----------
		wrench: a list of 6 elements 
		T: the task frame transform w.r.t. the EE frame

		Return:
		-----------
		"""
		return



	def _desiredVelocity(self):
		return

	def activate(self):
		self._sharedLock.acquire()
		self.state = 'active'
		self._sharedLock.release()

	def deactivate(self):
		self._sharedLock.acquire()
		self.state = 'idle'
		self.global_path_parent_conn.send(([],[],[],False,False))
		self._sharedLock.release()

	def setTrajectory(self,traj,w_traj,T,Y,kp = None,ki = None, kd = None,max_velocity = None,min_velocity = None):
		

		self._sharedLock.acquire()
		self.trajectory = traj
		self.total_time = trajectory
		self.current_time = 0.0
		self._sharedLock.release()


	##Below are functions that are not really used
	@classmethod
	def getVelocity(cls,t,transform):
		"""
		Return:
		----------
		Desired veloctiy of the task frame at time t, dimension k
		"""
		return

	@classmethod
	def getTaskFrameTransform(cls):
		"""
		Return:
		----------
		Get the task frame transform w.r.t the EE, lets assume R= I
		"""
		return ([1,0,0,0,1,0,0,0,1],[0,0,0])

	@classmethod
	def getWrench(cls,t):
		"""
		Return:
		---------
		Desired force/torque of the task frame at time t, dimension 6-k
		"""
		return

	@classmethod
	def getSelectionMatrices(cls):
		"""
		Return:
		--------
		T,Y: the desired selection matrix at the task frame for desired velocity and force, 
		6xK and 6x(6-k)
		"""
		T = np.array(([[]]))  #desired velocity
		Y = np.array(([[]]))  #desired force
		return T,Y

if __name__=="__main__":
	demo = PointClickNav(debugging = False)
	#time.sleep(10)
	#print('\n\n\n\n\n\n calling shutdown, boys! \n\n\n\n\n\n\n')
	#demo._shutdown()