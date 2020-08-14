import time,math
import json
import os
import datetime
import csv
from threading import Thread
import sys
import signal
from copy import copy,deepcopy
"""
When doing F/T calibration, make sure that the weight and cog are set to zero for the limb attachment, in the configuration file
"""

class Calibration:
	def __init__(self,Jarvis = None, components = ['left_limb'], debugging = False, codename = 'Anthrax',server_address = 'http://localhost:8080' ):
		self.mode = mode
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl+c)
		self.tasks = tasks
		self.codename = codename
		self.components = components

		if not Jarvis: #This means running this locally
			sys.path.append('../../Motion/')
			from motion_client import MotionClient
			#At this point the calibration data has not been loaded into Motion yet yet
			self.server_address = server_address
			self.robot =  MotionClient(address = server_address)
			self.startLocally()
		else:
			self.status = 'idle' #states are " idle, active"
			self.state = 'idle' #states are " idle, active
        	self.robot = Jarvis
			pass
			# stateRecieverThread = threading.Thread(target=self._serveStateReciever)
			# main_thread = threading.Thread(target = self._infoLoop)
			# stateRecieverThread.start()
			# main_thread.start()

	def sigint_handler(self, signum, frame):
		""" Catch Ctrl+C tp shutdown the api,
			there are bugs with using sigint_handler.. not used rn.
		"""
		assert(signum == signal.SIGINT)
		#logger.warning('SIGINT caught...shutting down the api!')
		print("SIGINT caught...shutting down the api!")

	def return_threads(self):
		return [self._serveStateReciever, self._infoLoop]

	def return_processes(self):
		return []

	def cameraCalibration(self,location,camera_guess,marker_guess):
		"""
		Parameters:
		-------------
		location: string, specifying the location of the camera, 'left' or 'right': wrist-mounted. 'fixed':mounted on the torso, or in the workspace.
		This assumes that the camera does not move w.r.t. the arm base after the calibration is performed.

		camera_guess: transform in Klampt format, initial guess, shouldn't be too far off the actual transform
		marker_guess: transform in Klampt format, initial guess. w.r.t. the wrist when calibrating a fixed camera, w.r.t. the torso, when calibrating a 
		wrist-mounted camera
		"""

		if not (location == 'fixed'):
			from take_calibration_pictures import take_pictures
			from process_pictures import process
			from calbration_calculation import calculation
			##TODO: load the camera module
			##TODO: load the configurations
			#camera = 
			#configs = 
			#the pictures will be saved to disk, for debugging purposes
			take_pictures(camera = camera,robot = self.robot,arm = location,configurations = configs)
			pts = process()
			transform = calculation(pts,camera_guess,marker_guess)


	def FTCalibration(self,arm,mass_guess,cog_guess):
		"""
		This commands the limb to move for 10 secs and use the data gathered to calculate te mass and cog, and update the system configuration file

		Parameters:
		------------
		arm: string, 'left' or 'right'
		mass_guess: float, the mass of the attachemnt
		cog_guess: a list of 3 floats, the initial guess of the attachment's center of gravity, in Klamp't EE frame
		"""

		from klampt.model.trajectory import Trajectory,SO3Trajectory
		from klampt.io import loader
		import autograd.numpy as np
		from autograd import grad
		import scipy.optimize as opt
		import scipy.linalg as lin
		from math import sqrt
		from klampt.math import so3

		#first collect data
		self.robot.startServer(mode = 'Physical',components = self.components,codename = self.codename)
		self.robot.startup()

		if arm == 'left':
			#TODO: change this after Vivek changes this
			initial_config = copy(TRINAConfig.left_untucked_config)
			self.robot.setLeftLimbPositionLinear(initial_config,5)
		elif arm == 'right':
			initial_config = copy(TRINAConfig.right_untucked_config)
			self.robot.setRightLimbPositionLinear(initial_config,5)	

		time.sleep(5)
		print('Robot Moved to Initial Config, start collecting data')

		def desired_w(t):
			assert t >= 0, "time should be positive"
			m1 = 11.0
			if t <= m1:
				return [math.pi/4.0/m1,-math.pi/3.0/m1,-math.pi/3.0/m1]
			else:
				return [0,0,0]

		current_time = 0
		dt = 0.02
		total_time = 10.0
		R_history = []
		t_history = []
		wrench_history = []
		wrench_global_history = []
		while current_time <= total_time:
			#only collect data when the EE is not accelerating
			if current_time > 1:
				if arm == 'left':
					(R,_) = robot.sensedLeftEETransform()
					wrench_global = robot.sensedLeftEEWrench(frame = 'global',format = 'raw')
					wrench = robot.sensedLeftEEWrench(frame = 'local', format = 'raw')
				elif arm == 'right':
					(R,_) = robot.sensedRightEETransform()
					wrench_global = robot.sensedRightEEWrench(frame = 'global',format = 'raw')
					wrench = robot.sensedRightEEWrench(frame = 'local', format = 'raw')

				R_history.append(R)
				t_history.append(current_time)
				wrench_global_history.append(wrench_global)
				wrench_history.append(wrench)
				w = desired_w(current_time)
				robot.setLeftEEVelocity([0,0,0]+w,tool = [0,0,0.0])
			current_time += dt
			time.sleep(dt)

		trajectory = SO3Trajectory(times = t_history,milestones = R_history)
		wrenches = Trajectory(times = t_history,milestones = wrench_history)
		wrenches_global = Trajectory(times = t_history,milestones = wrench_global_history)

		#same these for debgging perposes
		loader.save(trajectory,'auto','R_trajectory')
		loader.save(wrenches,'auto','wrenches')
		loader.save(wrenches_global,'auto','wrenches_global')

		#move the limb back
		robot.setLeftLimbPositionLinear(initial_config,5)
		time.sleep(5)
		robot.shutdown()

		##now calculate the calibration

		R_trajectory = trajectory
		w_trajectory = wrenches #EE frame
		R_base_global_left = np.array([[sqrt(0.5),-sqrt(0.5),0],\
							[sqrt(0.25),sqrt(0.25),-sqrt(0.5),],\
							[sqrt(0.25),sqrt(0.25),sqrt(0.5)]])##base in global
		R_global_base_left = R_base_global_left.T
		global Rs,ws
		Rs = np.array(R_trajectory.milestones)
		ws = np.array(w_trajectory.milestones)

		def to_matrix(a):
			return np.array([[a[0],a[3],a[6]],[a[1],a[4],a[7]],[a[2],a[5],a[8]]])
		#define the cost function and cost function gradient
		def error_func(x):
			"""
			x is [m,cog]
			"""
			global Rs,ws
			#calculate the offset due to zeroing the sensor at the beginning
			m = x[0]
			cog = x[1:4]
			R_0 = to_matrix(Rs[0]) ##EE in R_global_base_left
			gravity = np.array([[0],[0],[-m*9.81]])
			G_0_base = np.dot(R_global_base_left,np.array([0,0,-m*9.81])) #this is irrelevant to EE orientation
			R_base_EE = np.dot(R_0.T,R_base_global_left)
			G_0_EE = np.dot(R_base_EE,G_0_base)
			tau_0_EE = np.cross(cog,G_0_EE) #torque in the EE frame
			#tau_0_base = np.dot(R_base_EE.T,tau_0_EE)
			offset_f_EE = ws[0,0:3] - G_0_EE  #offset in the EE frame
			offset_tau_EE = ws[0,3:6] - tau_0_EE
			#offset in the EE frame
			offset = np.concatenate((offset_f_EE,offset_tau_EE))
			[N_of_pts,n] = np.shape(Rs)
			error = 0.0

			for i in range(N_of_pts):
				R = to_matrix(Rs[i]) #this is EE in global
				G_base = np.dot(R_global_base_left,np.array([0,0,-m*9.81]))
				R_base_EE = np.dot(R.T,R_base_global_left)
				G_EE = np.dot(R_base_EE,G_base)
				tau_EE = np.cross(cog,G_EE) #torque in the EE frame
				expected_f = G_EE + offset[0:3]
				expected_tau = tau_EE + offset[3:6]
				error += np.linalg.norm(np.concatenate((expected_f,expected_tau)) - ws[i])
			return error

		error_func_grad = grad(error_func)

		def callback_func(x):
			return

		x0 = np.array([mass_guess] + cog_guess)# np.zeros(4)
		res = opt.minimize(fun = error_func,x0 = x0,jac = error_func_grad)#,callback = callback_func)
		
		##TODO: extract the solution from res
		#calibrated_mass = 
		#calibrated_cog = 

		T_klampt_in_EE = [0,0,1.0,-sqrt(0.5),-sqrt(0.5),0,sqrt(0.5),-sqrt(0.5),0]
		calibrated_cog_UR = so3.apply(T_klampt_in_EE,calibrated_cog) 

		##TODO:now change the confgiuration file

				

	def _infoLoop(self):
		pass
		# while(True):
		# 	self.robot.log_health()
		# 	loop_start_time = time.time()
		# 	status = self.robot.getActivityStatus()

		# 	if(status == 'active'):
		# 		if(self.status == 'idle'):
		# 			print('\n\n\n\n starting up Direct-Tele Operation Module! \n\n\n\n\n')
		# 			self.status = 'active'
		# 			self.state = 'active'

		# 	elif(status == 'idle'):
		# 		if self.status == 'active':
		# 			self.state = 'idle'
		# 			self.status = 'idle'

		# 	elapsed_time = time.time() - loop_start_time
		# 	if elapsed_time < self.infoLoop_rate:
		# 		time.sleep(self.infoLoop_rate)
		# 	else:
		# 		time.sleep(0.001)

if __name__ == "__main__" :
    example = Example()