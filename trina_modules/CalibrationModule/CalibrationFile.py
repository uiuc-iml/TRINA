import time,math
import json
import os
import datetime
import csv
from threading import Thread
import sys
import signal
from copy import copy,deepcopy
import numpy as np
from klampt import WorldModel

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
"""
When doing F/T calibration, make sure that the weight and cog are set to zero for the limb attachment, in the configuration file
"""


class Calibration:
	def __init__(self,Jarvis = None, components = ['left_limb'], debugging = False, codename = 'Anthrax',server_address = 'http://localhost:8080' ):
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl+c)
		self.codename = codename
		self.components = components

		if not Jarvis: #This means running this locally
			sys.path.append('../../Motion/')
			from motion_client_python3 import MotionClient
			#At this point the calibration data has not been loaded into Motion yet

			self.server_address = server_address
			self.robot =  MotionClient(address = server_address)

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
		sys.path.append('../../')
		from SensorModule import Camera_Robot
		if not (location == 'fixed'):
			from take_calibration_pictures import take_pictures
			from process_pictures import process
			from calibration_calculation import calculation
			import TRINAConfig
			
			# self.robot.startServer(mode = 'Physical',components = self.components,codename = self.codename)
			# self.robot.startup()
			
			# #self.robot.setLeftLimbPositionLinear(TRINAConfig.left_untucked_config,10)
			# self.robot.setRightLimbPositionLinear(TRINAConfig.right_untucked_config,10)
			# time.sleep(10)

			# #the pictures will be saved to disk, for debugging purposes
			# EE_transforms = take_pictures(camera = [],robot = self.robot,arm = location,configurations = TRINAConfig.right_calibration_configs)
			# print(EE_transforms)
			# self.robot.shutdown()
			# pts = process(18)
			# T_camera,T_marker = calculation(pts,EE_transforms,camera_guess,marker_guess,'wrist')
			# print(T_camera,T_marker)

		else:
			from take_calibration_pictures import take_pictures
			from process_pictures import process
			from calibration_calculation import calculation
			import TRINAConfig
			# then create a sensor_module instance using the motionClient instance and the world we created
			sensor_module = Camera_Robot(robot = [],world = [], cameras =['zed_overhead'],ros_active = False, use_jarvis = False, mode = 'Physical')

			#move the robot to some initial config
			self.robot.startServer(mode = 'Physical',components = self.components,codename = self.codename)
			self.robot.startup()
			time.sleep(1)

			#self.robot.setLeftLimbPositionLinear(TRINAConfig.left_tabletop_config,10)
			#self.robot.setRightLimbPositionLinear(TRINAConfig.right_tabletop_config,10)
			#time.sleep(10)

			# #the pictures will be saved to disk, for debugging purposes
			#EE_transforms = take_pictures(camera = sensor_module,robot = self.robot,arm = location,configurations = TRINAConfig.fixed_calibration_configs,dimx = 1080,dimy = 1920)
			#print(EE_transforms)
			
			EE_transforms = [[[-0.6854497985866844, -0.34565019378414746, -0.6408467189230642, -0.5549872468933347, -0.32170860190912653, 0.7671327989621044, -0.47132550260221623, 0.8814927787535515, 0.028683646945803578], [0.6449667411885552, -0.366271439446788, 0.9373736599625151]], [[-0.6935727070168066, -0.3457430609770297, -0.6319957562101303, -0.5450746513738104, -0.32173887033507287, 0.774194887444519, -0.47101011083744165, 0.8814453103294314, 0.03469352082385776], [0.6039023748882635, -0.36745617122229607, 0.9427296304200445]], [[-0.6935598255152654, -0.34540797072093793, -0.6321930893276834, -0.5452042488505475, -0.3219362036096348, 0.774021580991594, -0.47087906671091884, 0.8815046310529865, 0.034964123978644575], [0.5619819082882999, -0.36857244920830223, 0.9438268676369935]], [[-0.7323166508799486, -0.3473435143457039, -0.585717342995709, -0.49370538446852263, -0.3215868623743696, 0.8079832196865211, -0.469006733630515, 0.8808713714216087, 0.0640180507282964], [0.5356362078245624, -0.3692438697173501, 0.9444786134713646]], [[-0.7322243317742939, -0.3468116909645126, -0.586147744999523, -0.4940333068396811, -0.32194313498750143, 0.8076408295569696, -0.46880552438163475, 0.8809507754947247, 0.06439807034768524], [0.4774589330923385, -0.370760083490538, 0.9459768866111562]], [[-0.7492372819014854, -0.33072378935851265, -0.5738164084105838, -0.4704398885809503, -0.34409459458589625, 0.8125793630216753, -0.46618645051182966, 0.8787608805181965, 0.1022228361476427], [0.47471175282220335, -0.30758649324391407, 0.9476886992745103]], [[-0.749266548031799, -0.33104134075778563, -0.5735950406949151, -0.47029527536844185, -0.34383919875430663, 0.8127711605157497, -0.46628531390837796, 0.8787412793930224, 0.10194003101863826], [0.5196655137398016, -0.3064366667865594, 0.9464877618265629]], [[-0.7493044081332639, -0.331528898130643, -0.5732638953015808, -0.4700070966303419, -0.3435898378937005, 0.8130432660155903, -0.466514986971669, 0.8786550022622439, 0.10163244526410398], [0.5902706136124449, -0.3045404877158943, 0.9445828769328838]], [[-0.7493435095216187, -0.33246638070616524, -0.5726695473289707, -0.46989499062638396, -0.3423656759076419, 0.8136242632471914, -0.46656511069951956, 0.8787786124277538, 0.1003252102788624], [0.6193571518258961, -0.24551883881309844, 0.9450257942550759]], [[-0.7492773731295743, -0.33239674665305174, -0.5727964917232937, -0.4699305463470514, -0.34257456017866245, 0.8135157972211472, -0.4666355105711605, 0.8787235478424745, 0.10047998179303694], [0.5812191708215921, -0.24650205314803844, 0.9460747227005646]], [[-0.7492197125824994, -0.33179704534641674, -0.5732194544650296, -0.4702128298540839, -0.34304824830811964, 0.8131529954278872, -0.46644369104792544, 0.878765395331496, 0.1010032823697773], [0.46895547403381793, -0.2494822990980679, 0.9491953186172164]], [[-0.7556874722915827, -0.16884198162777897, -0.6327944606754746, -0.4672736104995817, -0.5380072303791977, 0.7015722293466232, -0.4589028406452128, 0.8258574969688847, 0.3276699216407351], [0.46381424044541897, -0.13733271796321084, 0.9513285164308292]], [[-0.755736517301638, -0.16919019358569798, -0.6326428651389556, -0.4670928978906976, -0.5378483693969354, 0.701814331769542, -0.4590060360788981, 0.8258897080915185, 0.3274441157077734], [0.5894730042845173, -0.13387768845981174, 0.9477870721152571]], [[-0.7598435401661006, -0.034235915764157764, -0.64920389442735, -0.4671839075379008, -0.6656746831410844, 0.5819075637613206, -0.4520807350523429, 0.7454563154852252, 0.48981413893203146], [0.6163315451004223, 0.009861507446864579, 0.9488243775013702]], [[-0.7597470762634999, -0.03565677251064719, -0.6492403058060927, -0.46734766065581446, -0.6642721689944873, 0.583376936104692, -0.45207360483510056, 0.7466398596870416, 0.48801472901738774], [0.5821045473925139, 0.0945259060719128, 0.9501380898046703]], [[-0.816573826326268, 0.0844037728872331, -0.571036942133589, -0.4255131702291836, -0.7564814747782618, 0.4966631859508177, -0.39005862138964376, 0.6485458977124964, 0.6536378893851738], [0.5030358514316454, 0.09232066920338736, 0.9523544698592932]], [[-0.8188306447879983, 0.2052741790787036, -0.5360773139759198, -0.42786378004207043, -0.8408290675829422, 0.3315706030933861, -0.38268650470611504, 0.5008682366762397, 0.7763259937710782], [0.5001934455830704, 0.16494125855752612, 0.9526311159514204]], [[-0.8189912679206949, 0.20547887162003, -0.5357534287220317, -0.42763853135025603, -0.8411203814739737, 0.3311220173496078, -0.3825945498491187, 0.5002948502501681, 0.7767409305804056], [0.58309026134779, 0.1676153459488999, 0.9506265839709986]]]

			self.robot.shutdown()

			sensor_module.safely_close_all()
			pts = process(18,dimy = 1920)
			print(pts)
			T_camera,T_marker = calculation(pts,EE_transforms,camera_guess,marker_guess,'fixed')
			print(T_camera,T_marker)
		

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
		#first we need to make sure the relevant parameters in the configuration are reset
		#TODO: reset the relevant parameters

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
	calibration = Calibration(Jarvis = None, components = ['right_limb'], debugging = False, codename = 'bubonic',server_address = 'http://localhost:8080')
	# camera_guess = ([1,0,0,0,1,0,0,0,1],[0.0,0.0,0.5])
	# marker_guess = ([0,1,0,0,0,1,1,0,0],[0,0.05,-0.05])
	camera_guess = ([1,0,0,0,1,0,0,0,1],[0.5,-0.1,1.5])
	marker_guess = ([0,1,0,0,0,1,1,0,0],[-15.0,0.0,0])
	calibration.cameraCalibration(location = 'fixed',camera_guess = camera_guess,marker_guess = marker_guess)