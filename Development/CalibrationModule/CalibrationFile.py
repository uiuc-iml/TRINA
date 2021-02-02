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
		self.codename = codename
		self.components = components

		if not Jarvis: #This means running this locally
			sys.path.append('../../Motion/')
			from motion_client import MotionClient
			#At this point the calibration data has not been loaded into Motion yet

			self.server_address = server_address
			self.robot =  MotionClient(address = server_address)

		else:
			self.status = 'idle' #states are " idle, active"
			self.robot = self.jarvis.robot
			pass
			# stateRecieverThread = threading.Thread(target=self._serveStateReciever)
			# main_thread = threading.Thread(target = self._infoLoop)
			# stateRecieverThread.start()
			# main_thread.start()

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
		if not (location == 'fixed'):
			from take_calibration_pictures import take_pictures
			from process_pictures import process
			from calibration_calculation import calculation
			from Settings import trina_settings
			sensor_module = Camera_Robot(robot = [],world = [], cameras =['realsense_right','realsense_left'],ros_active = False, use_jarvis = False, mode = 'Physical')
			self.robot.startServer(mode = 'Physical',components = self.components,codename = self.codename)
			self.robot.startup()
			
			# # #self.robot.setLeftLimbPositionLinear(trina_settings.left_arm_config('untucked'),10)
			# # self.robot.setRightLimbPositionLinear(trina_settings.right_arm_config('untucked'),10)
			# # time.sleep(10)

			#the pictures will be saved to disk, for debugging purposes
			if location == 'right':
				EE_transforms = take_pictures(camera = sensor_module,robot = self.robot,arm = location,configurations = trina_settings.calibration_configs('right'), dimx = 480, dimy = 640)
			elif location == 'left':
				EE_transforms = take_pictures(camera = sensor_module,robot = self.robot,arm = location,configurations = trina_settings.calibration_configs('left'), dimx = 480, dimy = 640)

			print(EE_transforms)
			# EE_transforms = [[[-0.7917289873104166, -0.549840170409329, -0.26615972207804545, -0.5714009322249597, 0.512502206435056, 0.6409699392730112, -0.21602357580799053, 0.659558394233172, -0.7199420388418021], [0.4178302132699079, 0.19437048012716093, 0.9911301974975555]], [[-0.825647524264082, -0.5103571437691576, -0.2405031215605737, -0.5253129038144274, 0.5399169575709888, 0.6576747159602092, -0.205797275887334, 0.6693468941670496, -0.7138782925024957], [0.41680472588802353, 0.1479385261284853, 0.9913642616762413]], [[-0.9165854593776105, -0.3192901867919154, -0.2406758655867259, -0.39594610348056614, 0.6410216492880827, 0.6575088807632519, -0.05565769305400218, 0.6979577507001199, -0.7139728282235516], [0.41545688057986907, 0.10650827196258376, 0.9915643350630878]], [[-0.9913064162497566, -0.04272879719449945, -0.1244421110090026, -0.11620934691475802, 0.7278640597631655, 0.6758027065606712, 0.061700703332386, 0.6843888955815948, -0.7265017982172486], [0.41520073159821375, 0.10641013959237872, 0.9872158177825846]], [[-0.9887334615180722, 0.08266269881110298, -0.12479190798989051, -0.023694141084274364, 0.7367463190048645, 0.6757539856420097, 0.1477996270386731, 0.6710974144325462, -0.7264871166023401], [0.4149680382954846, 0.10638831935352641, 0.9872327201845177]], [[-0.9735934959638083, 0.20241781088779925, -0.10555915143825075, 0.07901067980489826, 0.732590893128899, 0.6760679668357099, 0.21417987088832618, 0.6498750750259576, -0.7292388975954774], [0.4147120366740249, 0.10635006851367175, 0.9872408503400782]], [[-0.975150508928164, 0.20283767884438444, 0.08909748020100183, 0.2115087549040788, 0.7327099010915257, 0.646838656421656, 0.06592064574980869, 0.649609942106392, -0.7574043778461146], [0.4151244555390717, 0.10631755544316915, 0.9873521517160951]], [[-0.9882302735159119, 0.124240098261568, 0.0892486666041782, 0.1523613310705291, 0.7472479690561752, 0.6468434876659415, 0.013673013606527869, 0.6528283623781821, -0.757382451588059], [0.41532047626111657, 0.10632504446739846, 0.9873461847203817]], [[-0.9971470998141592, -0.058110468728122505, 0.04817504287918336, -0.013177463831772145, 0.7624378280535736, 0.6469272855583638, -0.07432372285600097, 0.6444468417000343, -0.7610284176321388], [0.4156103787123706, 0.10637120704413347, 0.9872992888263155]], [[-0.9603572606827495, -0.27517860868813177, 0.04461687096144891, -0.16758675565793388, 0.6977851636335706, 0.6964269845008301, -0.22277479925246227, 0.6613415144487271, -0.7162393385487081], [0.416068192940999, 0.10643499710964618, 0.9872270180158992]], [[-0.9462838474876553, -0.31777266515163227, 0.05972782656676693, -0.20027549412676998, 0.7210640855151635, 0.6632920254551982, -0.2538436653458971, 0.6157005098768479, -0.7459733746596803], [0.4163846057236876, 0.10642998455384911, 0.9791595751562842]], [[-0.854548661263833, -0.519243938865801, 0.01150293368321187, -0.3916916121144334, 0.6588566210007752, 0.6422504448909618, -0.34106343476161316, 0.5443286552392501, -0.7664085389360185], [0.4167653095151046, 0.10653339512876843, 0.979051504159112]], [[-0.8935422028832465, -0.4443812614505655, 0.06409076484299932, 0.20854861669471833, -0.28438155252770136, 0.9357534969513365, -0.3976050881732733, 0.8495012813725117, 0.3467820162655288], [0.4073736399470138, 0.11810389148855116, 0.964898332979222]], [[-0.9092909526601384, -0.34268162572608357, 0.2361340017874344, 0.3195941587524359, -0.21157622291830028, 0.9236314609122787, -0.26655119039640907, 0.9153167786539014, 0.30190339119145915], [0.40753953564827267, 0.11806996531549485, 0.9649879400740283]], [[-0.8692678169749651, -0.34237060532141284, 0.35658916273974073, 0.4411095128133393, -0.21158824357459996, 0.872154122209255, -0.2231498601325093, 0.9154303817291986, 0.3349497815045952], [0.40785221802317073, 0.11808625880424672, 0.9650451590018426]], [[-0.8319637570532956, -0.3421423965084221, 0.43677784680686377, 0.521100338831018, -0.21155563013803277, 0.826860721178059, -0.1905012961109844, 0.9155232361160336, 0.3542971356244094], [0.4080481916195336, 0.11810045842577409, 0.9650394974018911]], [[-0.8048174913349033, -0.3419659784042262, 0.48510625151139275, 0.5689133465751294, -0.2115666800341466, 0.7947182796362605, -0.16913429495546253, 0.9155865931267071, 0.36482157386392217], [0.40817558635449525, 0.11813116127483803, 0.9650679452349662]], [[-0.8048778217506954, -0.3419607322766182, 0.4850098448843033, 0.5688579940754561, -0.2117935840529403, 0.7946974646558651, -0.16903335360308472, 0.9155360917707446, 0.3649950548087537], [0.40411052039944073, 0.13582578244807533, 0.9903283687089168]]]

			self.robot.shutdown()
			pts = process(18,location = location,dimy = 640)
			T_camera,T_marker = calculation(pts,EE_transforms,camera_guess,marker_guess,'wrist')
			print(T_camera,T_marker)
			


		else:
			from take_calibration_pictures import take_pictures
			from process_pictures import process
			from calibration_calculation import calculation
			from Settings import trina_settings
			# then create a sensor_module instance using the motionClient instance and the world we created
			# sensor_module = Camera_Robot(robot = [],world = [], cameras =['zed_overhead'],ros_active = False, use_jarvis = False, mode = 'Physical')
			sensor_module = 1
			#move the robot to some initial config
			# self.robot.startServer(mode = 'Physical',components = self.components,codename = self.codename)
			# self.robot.startup()
			# time.sleep(0.1)

			self.robot.setLeftLimbPositionLinear([-3.5516813437091272, -0.28219016016040044, 1.2513144651996058, 3.372593565578125, -0.8351472059832972, -4.003861252461569],10)
			self.robot.setRightLimbPositionLinear([3.4812798500061035, -2.6235443554320277, -1.6114587783813477, -0.05091269434008794, 0.9051432609558105, -0.3359759489642542],10)
			time.sleep(10.5)
			EE_transforms = take_pictures(camera = sensor_module,robot = self.robot,arm = location,configurations = trina_settings.calibration_configs('fixed'),dimx = 1080,dimy = 1920)
			self.robot.setLeftLimbPositionLinear([-3.5775955359088343, -0.3373478215983887, 1.3170684019671839, 3.3618618684956054, -0.5811975638019007, -4.019508186970846],10)
			self.robot.setRightLimbPositionLinear([3.5036206245422363, -2.5830193958678187, -1.6527585983276367, -0.03898556650195317, 0.45035600662231445, -0.3307307402240198],10)
			time.sleep(10.5)
			EE_transforms = take_pictures(camera = sensor_module,robot = self.robot,arm = location,configurations = trina_settings.calibration_configs('fixed'),dimx = 1080,dimy = 1920)
			self.robot.shutdown()

			time.sleep(2.0)
			initial_urdf_path = '../../Models/robots/Bubonic_uncalibrated.urdf'
			final_urdf_path = '../../Models/robots/Bubonic.urdf'
			# final_urdf_path ='../../../Downloads/tmp.urdf'
			from dense_calibration import extrinsic_calibration_read
			import glob
			root_path = '/data/test_data/'
			folder_list = glob.glob(root_path + 'calibration_*')
			if len(folder_list)>0:
				number_list = []
				for folder in folder_list:
					number = [int(s) for s in folder.split('_') if s.isdigit()]
					number_list.append(number[0])
				largest_number = np.max(number_list)
				path = root_path + 'calibration_' + str(largest_number)
			else:
				print('no folder detected')
				exit()

			c =    \
			extrinsic_calibration_read( folder= path,lPos=None,rPos=None,
						ROBOT_PATH= initial_urdf_path,
						ROBOT_PATH_OUT=final_urdf_path,
						desired_base_mesh=None,
						vis=True)
			

			print(c)

			# sensor_module.safely_close_all()
			# pts = process(13,location = location,dimy = 1920)
			# T_camera,T_marker = calculation(pts,Ts,camera_guess,marker_guess,location)
			# print(T_camera,T_marker)
		

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
	calibration = Calibration(Jarvis = None, components = ['right_limb','left_limb'], debugging = False, codename = 'bubonic',server_address = 'http://10.0.242.158:8080')
	# marker_guess = ([1,0,0,0,1,0,0,0,1],[0.0,0.0,0.9])
	# camera_guess = ([0,-1,0,0,0,1,-1,0,0],[0,0.05,-0.05])
	camera_guess = ([-0.0010836480924374516, 0.9999506012637995, -0.009880320793254236, 0.5781048518002231, -0.0074355414760572565, -0.8159286078132078, -0.8159617675062071,\
    -0.006596040867345257, -0.5780682366408164], [1.3029336845993607, -0.07284153359509085, 1.3922756366140938])
	marker_guess = ([1,0,0,0,1,0,0,0,1],[0,0.0,0])
	calibration.cameraCalibration(location = 'fixed',camera_guess = camera_guess,marker_guess = marker_guess)