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
from Motion  import MotionClient

class ForceControl:
	def __init__(self,EE_2_task= None):
		#use motion client to debug
		self.robot = MotionClient(address = 'http://localhost:8080')
		self.dt = 0.01
		self.EE_2_task = EE_2_task #z axis is the one pointing forward

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


	#Here we assume that 
	#There are a few different input formats to the force control algorithm:
	# - The most basic version, a function f(t) that gives T,Y,v_des,f_des
	# - Another version, where a desired EE trajectory and f trajectory is given, along with T,Y that are not time varying


	def runOnce(self,tt,command_type,T,Y,kp,kd,translation_Ds,rotation_Ds,pos_des = None,w_des = None,desired_speed = None,trajectory = None,w_trajectory = None):
		"""
		desired_speed: desired speed for translation and rotation

		translation_Ds/rotation_Ds: the corresponding dimensions in the short desired postion vector
		"""
		loop_start_time = time.time()
		self.current_time = 0.0
		(_,D_w) = np.shape(Y)
		(_,D_p) = np.shape(T)
		self.last_error = np.zeros(D_w)

		self.total_time = tt
		self.robot.startServer(mode = 'physical',components = ['left_limb'])
		self.command_type = command_type
		self.T = T
		self.Y = Y
		self.kp = kp
		self.kd = kd

		if self.command_type == 'point':
			self.pos_des = pos_des
			self.w_des = w_des
		elif self.command_type == 'trajectory':
			self.trajectory = trajectory
			self.w_trajectory = w_trajectory
		while self.current_time <= self.total_time:
			#get the current state, and EE transform
			current_EE_transform = self.robot.sensedLeftEETransform()
			current_task_transform = so3.mul(current_EE_transform,self.EE_2_task)
			current_wrench_EE = self.robot.sensedLeftEEWrench(frame = 'local')  #this one should have already been filtered
			current_q = self.robot.sensedLeftLimbPosition() 

			#TODO: check for safety
			if np.linalg.norm(current_wrench_EE[0:3]) > self.max_force:
				#self.resetParameters()
				self.jarvis.setLeftLimbPosition(current_q) #this is less computationally heavy than set the v = 0
				break

			#get wrench in the task frame 
			current_wrench = np.array(ForceControl.EEtoTask(current_wrench_EE,self.EE_2_task)) 
			if self.command_type == 'point':
				difference = self.pos_des - self.T.T@np.array(current_task_transform[1] + so3.moment(current_task_transform[0]))
				trans_diff = difference[translation_Ds]
				rotation_diff = difference[rotation_Ds]

				if trans_diff:
					distance = np.linalg.norm(trans_diff)
					if distance < 1.0*self.dt*desired_speed[0]
						v_des_trans = trans_diff/distance*desired_speed[0]
					else:
						v_des_trans = trans_diff/self.dt
				else:
					v_des_trans = np.array([])

				if rotation_diff:
					distance = np.linalg.norm(rotation_diff)
					if distance < 1.0*self.dt*desired_speed[1]
						v_des_rotation = rotation_diff/distance*desired_speed[1]
					else:
						v_des_rotation = rotation_diff/self.dt
				else:
					v_des_rotation = np.array([])	

				v_des = np.concatenate((v_des_trans,v_des_rotation))
				w_des = copy(self.w_des)

			elif self.command_type == 'trajectory':
				target_point = self.trajectory.eval(self.current_time)
				v_des = (np.array(target_point) - self.T.T@np.array(current_task_transform[1] + so3.moment(current_task_transform[0])))/self.dt
				w_des = np.array(self.w_trajectory.eval(self.current_time))
					
			self.last_error = copy(self.wrench_error)
			self.wrench_error = w_des - self.Y@np.array(current_wrench)
			d_error = (self.wrench_error-self.last_error)/self.dt	
			v_des_from_wrench = np.multiply(self.kp,self.wrench_error) + np.multiply(self.kd,d_error) #PID gains
			v_des_total = self.T@v_des + self.Y@v_des_from_wrench #transform back to the 6D velocity vector

			#clip the velocities for safety
			v_des_total = np.clip(v_des_total,self.min_velocity,self.max_velocity)
			#send the velocity to motion
			self.robot.setLeftEEVelocity(v_des_total.tolist(), tool = self.EE_2_task[1])
			self.current_time += self.dt
		
			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.dt:
				time.sleep(-elapsed_time + self.dt)
			else:
				time.sleep(0.0001)

	
		
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
		force_task = np.array(wrench[0:3])
		torque_task = np.array(rewnch[3:6]) - np.cross(np.array(T_task[1]),force_task)
		return np.concatenate((force_task,torque_task))

	def resetParameters(self):
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
		return

	def shutdonw(self):
		self.robot.shutdown()
		return


if __name__=="__main__":
	EE_2_task = ([[1,0,0,0,1,0,0,0,1],[0,0,0.05])
	control = ForceControl(EE_2_task)
	#worked for scanning vein at 10 mm/s
	# kp=0.003
	# kd=0.0012


	T = np.array([[1,0,0,0,0],\
		[0,1,0,0,0],
		[0,0,0,0,0],
		[0,0,1,0,0],
		[0,0,0,1,0],
		[0,0,0,0,1]])

	Y = np.array([[0],\
		[0],
		[1],
		[0],
		[0],
		[0]])

	kp = np.array([0.003])
	kd = np.array([0.0012])
	trans_Ds = [0,1]
	rotation_Ds = [2,3,4]

	###point mode
	#TODO find this out
	pos_des = np.array([])
	w_des = np.array([2])
	control.runOnce(tt = 10,command_type = 'point',T = T,Y = Y,kp = kp,kd = kd,translation_Ds = trans_Ds,\
		rotation_Ds = rotation_Ds,pos_des = pos_des,w_des = w_des)

	###trajectory mode
	
	control.shutdonw()