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
			# EE_transforms = take_pictures(camera = sensor_module,robot = self.robot,arm = location,configurations = TRINAConfig.fixed_calibration_configs,dimx = 1080,dimy = 1920)
			# print(EE_transforms)
			
			EE_transforms =[[[-0.6855169892074306, -0.34567562106928473, -0.640761127493191, -0.5548868801682069, -0.32171542660520286, 0.7672025381223835, -0.4713459533348889, 0.8814803170137147, 0.02873052370264049], [0.6449106666792422, -0.3662114317917563, 0.9373127130885679]], [[-0.6935393207936557, -0.34574917623344004, -0.632029048103783, -0.5451314926713867, -0.3217015395714479, 0.7741703786216747, -0.4709934885016376, 0.8814565369723337, 0.03463390272261827], [0.6039279368624167, -0.3674746208252079, 0.9427607277304092]], [[-0.6935490567291858, -0.34542927001329243, -0.6321932658041703, -0.5451980607059794, -0.32194639155933524, 0.7740217022567024, -0.47090209227874247, 0.8814925639958681, 0.03495824806101307], [0.5619860852043007, -0.3685656417871357, 0.943828130202346]], [[-0.7322989450439183, -0.34732763521890886, -0.5857488957742957, -0.49374511814059513, -0.32158001134684044, 0.8079616665501506, -0.46899255153988917, 0.8808801337954038, 0.06400137877184904], [0.535658031621073, -0.36924939937513057, 0.944496566413607]], [[-0.7322646978401959, -0.3468018708927292, -0.5861031262860759, -0.4939497780821684, -0.3220091042551279, 0.8076656198634352, -0.4688304907208842, 0.8809305302474847, 0.06449319226322954], [0.4774214334353201, -0.3707448858998752, 0.9459734792270122]], [[-0.7492394438921457, -0.33074947188043247, -0.5737987822982943, -0.4703909051992953, -0.34414478164258483, 0.8125864665215424, -0.4662324013000138, 0.8787315608984321, 0.10226530134424905], [0.47470047465597787, -0.30755193327920216, 0.9476778356913302]], [[-0.7492737802301987, -0.3310341294579714, -0.5735897553073561, -0.4703017658936486, -0.3438178716740967, 0.8127764268941433, -0.46626714580470385, 0.8787523406805239, 0.10192778076480148], [0.5196658402025984, -0.30643201125944797, 0.9464840742038038]], [[-0.7492807182528342, -0.33154654981683557, -0.573284650552473, -0.47003796652808366, -0.3435699415473894, 0.8130338278861886, -0.46652193440036854, 0.8786561219098618, 0.1015908664874052], [0.5902830429186092, -0.3045330800611138, 0.9445800240058739]], [[-0.7493537628366197, -0.33249674856098965, -0.572638498809651, -0.4698506754835519, -0.342378555509454, 0.8136444355336736, -0.4665932713512382, 0.8787621048518376, 0.10033883697811084], [0.6193248702712222, -0.24550863554957336, 0.9449991505328189]], [[-0.7492642803566334, -0.3324160667449735, -0.5728024063771551, -0.4699397981322222, -0.34256471480524286, 0.8135145987023527, -0.4666472161205979, 0.878720077578865, 0.10045596521208597], [0.5812133012564531, -0.24649743031102148, 0.9460802688787888]], [[-0.749245286745163, -0.3318093703175313, -0.573178891847601, -0.4701617641860578, -0.34305861765991996, 0.8131781479766402, -0.46645408754518786, 0.8787566936409361, 0.10103097343705138], [0.4689316374512733, -0.24947894098336532, 0.9491810085944202]], [[-0.7556692860118447, -0.16883103237292996, -0.6328190994954568, -0.4673274774419473, -0.5379587109950419, 0.7015735557247719, -0.4588779347561796, 0.8258913413836264, 0.3276194945687146], [0.46382996125852927, -0.13732978662562925, 0.951332194602049]], [[-0.7557254305832052, -0.16917255732978018, -0.6326608249420322, -0.4671066641206929, -0.537865292579707, 0.7017921995663566, -0.4590102808258965, 0.8258822996557383, 0.32745685091548077], [0.589494280362114, -0.13387641793865948, 0.9477841966835497]], [[-0.7598558980317865, -0.03420838181230667, -0.6491908816673968, -0.4672085228443121, -0.6656405044274665, 0.5819268983705639, -0.4520345234691088, 0.7454880988180383, 0.48980841572264205], [0.6163364685464979, 0.009849999843034278, 0.9488053192793244]], [[-0.7597360090837253, -0.03565537558746015, -0.649253333216898, -0.467352096073748, -0.6642825222544038, 0.583361593630231, -0.4520876184955847, 0.7466307151591386, 0.48801573784453556], [0.5821079565481058, 0.0945281256320534, 0.9501464754217168]], [[-0.8165927398000209, 0.08440242513658576, -0.571010094426499, -0.4254767011264165, -0.7565135867635019, 0.496645517286529, -0.39005880852055924, 0.6485086149568766, 0.6536747679253329], [0.5030137350575753, 0.09231714297746382, 0.9523462499892132]], [[-0.8188212045345692, 0.2053191802129197, -0.5360744997117964, -0.4279049780459565, -0.8408106202369299, 0.3315642180035109, -0.3826606391674585, 0.5008807593963713, 0.7763306641492841], [0.5002229282512551, 0.16491495545544063, 0.9526428936379989]], [[-0.8190029533956704, 0.20548805026617464, -0.5357320445213041, -0.42761611358450713, -0.841145923817425, 0.3310860828367955, -0.38259459187507183, 0.5002481344724218, 0.7767709972860799], [0.5830908603597148, 0.1676183439622174, 0.9506272561608005]]]

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