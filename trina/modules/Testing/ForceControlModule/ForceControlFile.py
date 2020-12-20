import os,sys
import signal
import time
import math
from threading import Thread, Lock, RLock
import threading
import numpy as np
import logging
from copy import deepcopy,copy
from klampt.math import vectorops,so3
from klampt import WorldModel, vis
from TRINAConfig import *
from motion import Motion
import jarvis

class ForceControlModule(jarvis.Module):
	def __init__(self,Jarvis = None,EE_2_task= None):
		#if true, run a test locally, otherwise, communicate with Jarvis to get state
		jarvis.Module.__init__(self.Jarvis)
		self.status = 'idle'
		self.state = 'idle'
		self.infoLoop_rate = 0.02
		self.exit_flag = False
		self.dt = 0.01

		self.EE_2_task = EE_2_task

		#For safety
		self.max_velocity = np.array([0.3,0.3,0.3,0.5,0.5,0.5])
		self.min_velocity = -np.array([0.3,0.3,0.3,0.5,0.5,0.5])
		self.max_force = 20

		#parameters for PD controller
		self.kp = None
		self.kd = None
		
		self.max_force = None
		self.current_time = 0.0
		self.total_time = 0.0
		self.T = None
		self.Y = None
		self.command_type = None

		self.wrench_error = None
		self.last_error = None
		#self.accumulated_error = None #doing only PD

		#for 'point' type
		self.pos_des = None
		self.w_des = None
		#for 'trajectory' type
		self.trajectory = None
		self.w_trajectory = None
		#warm up the sensor

		self._sharedLock = RLock()
		main_thread = threading.Thread(target = self._mainLoop)
		state_thread = threading.Thread(target = self._infoLoop)
		state_thread.start()
		main_thread.start()

	def return_threads(self):
		return [self._infoLoop,self._mainLoop]

	def _infoLoop(self):
		while not self.exit_flag:
			self.jarvis.log_health()
			loop_start_time = time.time()
			#check if there is new information
			
			status = self.jarvis.getActivityStatus()
			if(status == 'active'):
				if(self.status == 'idle'):
					print('\n\n\n\n starting up Autonomous Navigation Module! \n\n\n\n\n')
					self.activate()
			elif(status == 'idle'):
				if self.status == 'active':
					self.deactivate()

			if self.state == 'executing':
				if self.jarvis.cartesianDriveFail():
					print("singular configuration encountered")
					#TODO do something here to handle the case where cartesian drive failed


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
			self._sharedLock.acquire()

			loop_start_time = time.time()
			self.last_timestamp = time.time()
			if self.state == 'idle':							
				pass
			elif self.state == 'active':
				#self.current_time = 0.0
				#self.state = 'executing'
				pass
			
			if self.state == 'executing':
				if self.current_time <= self.total_time:
					#get the current state, and EE transform
					current_EE_transform = self.jarvis.sensedLeftEETransform()
					current_task_transform = so3.mul(current_EE_transform,self.EE_2_task)
					current_wrench_EE = self.jarvis.sensedLeftEEWrench(frame = 'local')  #this one should have already been filtered
					current_q = self.jarvis.sensedLeftLimbPosition() 
					#TODO: check for safety
					if np.linalg.norm(current_wrench_EE[0:3]) > self.max_force:
						self.resetParameters()
						self.jarvis.setLeftLimbPosition(current_q) #this is less computationally heavy than set the v = 0
						pass

					current_wrench = np.array(ForceControl.EEtoTask(current_wrench_EE,self.EE_2_task)) #get wrench in the task frame 
					if self.command_type == 'point':
						v_des = self.pos_des - self.T.T@np.array(current_task_transform[1] + so3.moment(current_task_transform[0]))
						w_des = copy(self.w_des)
					elif self.command_type == 'trajectory':
						target_point = self.trajectory.eval(self.current_time)
						v_des = np.array(target_point) - self.T.T@np.array(current_task_transform[1] + so3.moment(current_task_transform[0]))
						w_des = np.array(self.w_trajectory.eval(self.current_time))
					
				self.last_error = copy(self.wrench_error)
				self.wrench_error = w_des - self.Y@np.array(current_wrench)
				d_error = (self.wrench_error-self.last_error)/self.dt	
				v_des_from_wrench = np.dot(self.kp,self.wrench_error) + np.dot(self.kd,d_error) #PID gains
				v_des_total = self.T.T@v_des + self.Y.T@v_des_from_wrench #the 6D velocity vector

				#clip the velocities for safety
				v_des_total = np.clip(v_des_total,self.min_velocity,self.max_velocity)
				#send the velocity to motion
				self.jarvis.setLeftEEVelocity(v_des_total.tolist(), tool = self.EE_2_task[1])
				self.current_time += self.dt
			self._sharedLock.release()
			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.dt:
				time.sleep(-elapsed_time + self.dt)
			else:
				time.sleep(0.0001)

	def _shutdown(self):
		self._sharedLock.acquire()
		self.exit_flag = True
		self._sharedLock.release()
		return
	
		
	@classmethod
	def EEtoTask(cls,wrench,T_task):
		"""
		Parameters:
		-----------
		wrench: a list of 6 elements 
		T: the task frame transform w.r.t. the EE frame, right now we assume T[0] == I

		Return:
		-----------
		"""
		#TODO
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
		self.resetParameters()
		current_q = self.jarvis.sensedLeftLimbPosition() 
		self.jarvis.setLeftLimbPosition(current_q) #this is less computationally heavy than set the v = 0
		self._sharedLock.release()

	def setTrajectory(self,traj,w_traj,T,Y,kp = None,kd = None):
		self._sharedLock.acquire()
		self.trajectory = deepcopy(traj)
		self.w_traj = deepcopy(w_traj)
		#self.total_time
		#force_dim = 
		self.T = copy(T)
		self.Y = copy(Y)
		#TODO, check for consistency
		#assert
		
		#TODO: finish this up
		if kp == None:
			self.kp = np.array([1]*force_dim)
			self.kd = np.array([0.1]*force_dim)
		else:
			self.kp = copy(kp)
			self.kd = copy(kd)

		self.current_time = 0.0
		self.command_type = 'trajectory'
		self.wrench_error = np.zeros(force_dim)
		self.last_error = np.zeros(force_dim)
		self._sharedLock.release()

		return

	#TODO
	def setTargetPoint(self):
		return

	def resetParameters(self):
		self._sharedLock.acquire()
		#parameters for PD controller
		self.kp = None
		self.kd = None
		self.current_time = 0.0
		self.total_time = 0.0
		self.T = None
		self.Y = None
		self.command_type = None
		self.wrench_error = None
		self.last_error = None
		#for 'point' type
		self.pos_des = None
		self.w_des = None
		#for 'trajectory' type
		self.trajectory = None
		self.w_trajectory = None
		self._sharedLock.release()
		return

	def setMaxMinVel(self,maxVel,minVel):
		#TODO
		return


if __name__=="__main__":
	pass