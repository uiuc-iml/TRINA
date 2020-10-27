import time,math,datetime
import threading
# from motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import numpy as np
from scipy.spatial.transform import Rotation as R
import os,csv,sys,shlex
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback
from multiprocessing import Pool, TimeoutError,Process
import trina_modules
import sys, inspect
import atexit
import subprocess
from TRINAConfig import *
from SensorModule import Camera_Robot
from klampt import WorldModel,Geometry3D
import rosgraph
if(sys.version_info[0] < 3):
	from future import *
	from Motion import MotionClient
else:
	from Motion  import MotionClient
	from importlib import reload
from Jarvis import Jarvis
import redis
import traceback

robot_ip = 'http://localhost:8080'

ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

class CommandServer:

	#def __init__(self,components =  ['base','left_limb','right_limb','left_gripper'], robot_ip = robot_ip, model_name = model_name,mode = 'Kinematic',world_file = './Motion/data/TRINA_world_anthrax_PointClick.xml',modules = [],codename = 'anthrax_lowpoly'):
	def __init__(self,components =  ['base','left_limb','right_limb','left_gripper'], robot_ip = robot_ip, model_name = model_name,mode = 'Kinematic',world_file = './Motion/data/TRINA_world_bubonic.xml',modules = [],codename = 'bubonic'):
		# we first check if redis is up and running:
		try:
			self.interface = RedisInterface(host="localhost")
			self.interface.initialize()
			self.server = KeyValueStore(self.interface)
			print('Reem already up and running, skipping creation process')
		except Exception as e:
			# if we cannot connect to redis, we start the server for once:
			print('starting redis server because of ',e)
			self.redis_process = Process(target = self.start_redis())
			self.redis_process.daemon = False
			self.redis_process.start()
			# self.start_redis()
			# wait for it to start
			time.sleep(2)
			# then we start our connections as normal:
			self.interface = RedisInterface(host="localhost")
			self.interface.initialize()
			self.server = KeyValueStore(self.interface)

		self.start_ros_stuff()
		self.world_file = world_file
		# we then proceed with startup as normal
		self.always_active = set(['UI','devel','debug','DirectTeleoperation'])
		self.interface = RedisInterface(host="localhost")
		self.interface.initialize()
		self.server = KeyValueStore(self.interface)
		# self.server["ROBOT_STATE"] = 0
		self.server['ROBOT_COMMAND'] = {}
		self.server['HEALTH_LOG'] = {}
		self.server['ACTIVITY_STATUS'] = {}
		self.mode = mode
		self.components = components
		self.init_robot_state = {}
		self.dt = 0.001
		self.robot = MotionClient(address = robot_ip)
		# self.controller = UIController()
		self.robot.restartServer(mode = self.mode, components = self.components,codename = codename)
		self.robot_state = {}
		self.robot_command = {}
		self.modules = modules
		self.startup = True
		self.robot_active = True
		self.shut_down_flag = False
		self.left_limb_active = ('left_limb' in self.components)
		self.right_limb_active = ('right_limb' in self.components)
		self.base_active = ('base' in self.components)
		self.left_gripper_active = ('left_gripper' in self.components)
		self.right_gripper_active = ('right_gripper' in self.components)
		self.torso_active = ('torso' in self.components)
		self.query_robot = MotionClient(address = robot_ip)
		# self.controller = UIController()
		self.query_robot.startServer(mode = self.mode, components = self.components,codename = codename)
		self.query_robot.startup()
		res = self.robot.startup()
		if not res:
			print('Failed!')

		print('\n\n\n\n\n\n\n Initializing robot states')
		self.init_robot_states()
		print('\n\n\n\n\n\n initialized robot states sucessfully!')
		time.sleep(1)
		if(self.mode == 'Kinematic'):
			self.world = WorldModel()
			self.world.readFile(self.world_file )
			self.sensor_module = Camera_Robot(robot = self.robot,world = self.world)
			print('\n\n\n\n\n initialization of Kinematic sensor module sucessfull!!\n\n\n')

			time.sleep(3)
		if(self.mode == 'Physical'):
			self.sensor_module = Camera_Robot(robot = self.robot, mode = self.mode)
			print('\n\n\n\n\n initialization of Physical sensor module sucessfull!!\n\n\n')
		self.health_dict = {}
		# create the list of threads
		self.modules_dict = {}

		manager = Manager()

		self.active_modules = manager.dict()
		for i in self.always_active:
			self.active_modules[i] = True
		self.start_modules(self.modules,startup = True)

		stateRecieverThread = threading.Thread(target=self.stateReciever)
		stateRecieverThread.start()
		commandRecieverThread = Process(target=self.commandReciever, args=(self.robot, self.active_modules))
		commandRecieverThread.daemon = True
		commandRecieverThread.start()
		moduleMonitorThread = threading.Thread(target=self.moduleMonitor)
		moduleMonitorThread.start()
		atexit.register(self.shutdown_all)
		while(True):
			time.sleep(200)
		# self.switch_module_activity(['C2'])
		# self.empty_command.update({'UI':[]})

	def init_robot_states(self):
		pos_left = [0,0,0,0,0,0]
		pos_right = [0,0,0,0,0,0]
		pos_base = [0,0,0]
		pos_left_gripper = {}
		pos_right_gripper = {}
		pos_torso = {}
		vel_base = [0,0]
		vel_right = [0,0,0,0,0,0]
		vel_left = [0,0,0,0,0,0]
		posEE_left = {}
		velEE_left = {}
		posEE_right = {}
		velEE_right = {}

		# try:
		if(self.left_limb_active):
			posEE_left = self.query_robot.sensedLeftEETransform()
			pos_left = self.query_robot.sensedLeftLimbPosition()
			vel_left = self.query_robot.sensedLeftLimbVelocity()
			velEE_left = self.query_robot.sensedLeftEEVelocity()
			global_EEWrench_left = self.query_robot.sensedLeftEEWrench('global')
			local_EEWrench_left = self.query_robot.sensedLeftEEWrench('local')


		if(self.right_limb_active):
			posEE_right = self.query_robot.sensedRightEETransform()
			pos_right = self.query_robot.sensedRightLimbPosition()
			vel_right = self.query_robot.sensedRightLimbVelocity()
			velEE_right = self.query_robot.sensedRightEEVelocity()
			global_EEWrench_right = self.query_robot.sensedRightEEWrench('global')
			local_EEWrench_right = self.query_robot.sensedRightEEWrench('local')


		if(self.base_active):
			pos_base = self.query_robot.sensedBasePosition()
			vel_base = self.query_robot.sensedBaseVelocity()

		# print( self.query_robot.sensedLeftLimbPosition(),self.query_robot.sensedRightLimbPosition())
		klampt_q = get_klampt_model_q(codename,left_limb = self.query_robot.sensedLeftLimbPosition(), right_limb = self.query_robot.sensedRightLimbPosition(), base = pos_base)
		klampt_command_pos = self.query_robot.getKlamptCommandedPosition()
		klampt_sensor_pos = self.query_robot.getKlamptSensedPosition()
		# print("base velocity")
		if(self.left_gripper_active):
			pos_left_gripper = self.robot.sensedLeftGripperPosition()
		if(self.right_gripper_active):
			pos_right_gripper = self.robot.sensedRightGripperPosition()
		if(self.torso_active):
			pos_torso = self.robot.sensedTorsoPosition()

		self.server["ROBOT_INFO"] = {
			"Started" : self.query_robot.isStarted(),
			"Shutdown" : self.query_robot.isShutDown(),
			"Moving" : True, #self.query_robot.moving(),
			"CartesianDrive" : False,#self.query_robot.cartesianDriveFail(),
			"Components" : self.components,
			"Mode" : self.mode
		}
		# self.server["WORLD"] = self.world
		self.server["ROBOT_STATE"] = {
			"Position" : {
				"LeftArm" : pos_left,
				"RightArm" : pos_right,
				"Base" : pos_base,
				"Torso": pos_torso,
				"LeftGripper" : pos_left_gripper,
				"RightGripper" : pos_right_gripper,
				"Robotq": klampt_q
			},
			"PositionEE": {
				"LeftArm" : posEE_left,
				"RightArm" : posEE_right
			},
			"EEWrench":{
				"LeftArm" :{
					"global":global_EEWrench_left,
					"local": local_EEWrench_left
				},
				"RightArm" :{
					"global":global_EEWrench_right,
					"local": local_EEWrench_right
				}
			},
			"Velocity" : {
				"LeftArm" : vel_left,
				"RightArm" : vel_right,
				"Base" : vel_base
			},
			"VelocityEE" : {
				"LeftArm" : velEE_left,
				"RightArm" : velEE_right
			},
			"KlamptCommandPos" : klampt_command_pos,
			"KlamptSensedPos" : klampt_sensor_pos
		}
		self.server['TRINA_TIME'] = time.time()
		# except Exception as e:
		# 	print(e)


	def start_module(self,module,name):
		if(name != 'sensor_module'):
			module_trina_queue = TrinaQueue(str(name))
			module_jarvis = Jarvis(str(name),self.sensor_module,module_trina_queue)
			a = module(module_jarvis)
			return a.return_processes()

	def start_modules(self,module_names = [],startup = False):
		import trina_modules
		trina_modules = reload(trina_modules)
		activity_dict = {}
		command_dict = {}
		try:
			if(startup):
				if(module_names == []):
					print('\n\n Starting ALL modules available!')
					for name, obj in inspect.getmembers(trina_modules):
						if inspect.isclass(obj):
							if(str(obj).find('trina_modules') != -1):
								tmp = self.start_module(obj,name)
								self.modules_dict.update({name:tmp})
								self.health_dict.update({name:[True,time.time()]})
								activity_dict.update({name:'idle'})
								command_dict.update({name:[]})
								if(name not in self.always_active):
									self.active_modules[name] = False
								else:
									self.active_modules[name] = True

					self.server['HEALTH_LOG'] = self.health_dict
					self.server['ACTIVITY_STATUS'] = activity_dict
					self.empty_command = command_dict
				else:
					print('\n\n Starting Only Modules:' + str(module_names) + '\n\n\n')
					for name, obj in inspect.getmembers(trina_modules):
						if inspect.isclass(obj):
							if(str(obj).find('trina_modules') != -1):
								if(name in module_names):
									tmp = self.start_module(obj,name)
									self.modules_dict.update({name:tmp})
									self.health_dict.update({name:[True,time.time()]})
									activity_dict.update({name:'idle'})
									command_dict.update({name:[]})
									if(name not in self.always_active):
										self.active_modules[name] = False
									else:
										self.active_modules[name] = True

					self.server['HEALTH_LOG'] = self.health_dict
					self.server['ACTIVITY_STATUS'] = activity_dict
					self.empty_command = command_dict
			else:
				print('starting only modules '+ str(module_names))
				for name, obj in inspect.getmembers(trina_modules):
					if inspect.isclass(obj):
						if(str(obj).find('trina_modules') != -1):
							if(name in module_names):
								print('killing module '+ name)
								for pcess in self.modules_dict[name]:
									pcess.terminate()
								self.modules_dict.update({name:[]})
								print('restarting only module ' + name)
								tmp = self.start_module(obj,name)
								self.modules_dict.update({name:tmp})
								self.server['HEALTH_LOG'][name] = [True,time.time()]
								self.server['ACTIVITY_STATUS'][name] = 'idle'
								if(self.active_modules[name]):
									self.active_modules[name] = False
		except Exception as e:
			print('Failed to initialize module',name,'due to ',e)
			traceback.print_exc()
	def switch_module_activity(self,to_activate,to_deactivate = []):
		print('switching module activity:')
		if(to_deactivate == []):
			tmp = self.server['ACTIVITY_STATUS'].read()
			for i in tmp.keys():
				# print(i)
				self.server['ACTIVITY_STATUS'][str(i)] = 'idle'
				if(self.active_modules[i]):
					self.active_modules[i] = False
		else:
			print('gets here')

			for i in to_deactivate:
				self.server['ACTIVITY_STATUS'][i] = 'idle'
				if(self.active_modules[i]):
					self.active_modules[i] = False
		for i in to_activate:
			self.server['ACTIVITY_STATUS'][i] = 'active'
			self.active_modules[i] = True

	def shutdown_all(self):
		self.shutdown()
		print('closing all and exiting')
		for module in self.modules_dict.keys():
			for pcess in self.modules_dict[module]:
				try:
					pcess.terminate()
				except Exception as e:
					print(e)
					pass
	#this is place holder for moduleMonitor
	def activate(self,name):
		while not self.shut_down_flag:
			time.sleep(0.1)

	def stateReciever(self):
		pos_left = [0,0,0,0,0,0]
		pos_right = [0,0,0,0,0,0]
		pos_base = [0,0,0]
		pos_left_gripper = {}
		pos_right_gripper = {}
		pos_torso = {}
		vel_base = [0,0]
		vel_right = [0,0,0,0,0,0]
		vel_left = [0,0,0,0,0,0]
		posEE_left = {}
		velEE_left = {}
		posEE_right = {}
		velEE_right = {}
		loopStartTime = time.time()
		while not self.shut_down_flag:
			# print('updating states')
			try:
				if(self.left_limb_active):
					posEE_left = self.query_robot.sensedLeftEETransform()
					pos_left = self.query_robot.sensedLeftLimbPosition()
					vel_left = self.query_robot.sensedLeftLimbVelocity()
					velEE_left = self.query_robot.sensedLeftEEVelocity()

				if(self.right_limb_active):
					posEE_right = self.query_robot.sensedRightEETransform()
					pos_right = self.query_robot.sensedRightLimbPosition()
					vel_right = self.query_robot.sensedRightLimbVelocity()
					velEE_right = self.query_robot.sensedRightEEVelocity()

				if(self.base_active):
					pos_base = self.query_robot.sensedBasePosition()
					vel_base = self.query_robot.sensedBaseVelocity()

				klampt_q = get_klampt_model_q(codename,left_limb = pos_left, right_limb = pos_right, base = pos_base)
				klampt_command_pos = self.query_robot.getKlamptCommandedPosition()
				klampt_sensor_pos = self.query_robot.getKlamptSensedPosition()
				if(self.left_gripper_active):
					pos_left_gripper = self.robot.sensedLeftGripperPosition()
				if(self.right_gripper_active):
					pos_right_gripper = self.robot.sensedRightGripperPosition()
				if(self.torso_active):
					pos_torso = self.robot.sensedTorsoPosition()

				self.server["ROBOT_INFO"] = {
					"Started" : self.query_robot.isStarted(),
					"Shutdown" : self.query_robot.isShutDown(),
					"Moving" : True,#self.query_robot.moving(),
					"CartesianDrive" : True,#self.query_robot.cartesianDriveFail(),
					"Components" : self.components,
					"Mode" : self.mode
				}
				# self.server["WORLD"] = self.world
				self.server["ROBOT_STATE"] = {
					"Position" : {
						"LeftArm" : pos_left,
						"RightArm" : pos_right,
						"Base" : pos_base,
						"Torso": pos_torso,
						"LeftGripper" : pos_left_gripper,
						"RightGripper" : pos_right_gripper,
						"Robotq": klampt_q
					},
					"PositionEE": {
						"LeftArm" : posEE_left,
						"RightArm" : posEE_right
					},
					"Velocity" : {
						"LeftArm" : vel_left,
						"RightArm" : vel_right,
						"Base" : vel_base
					},
					"VelocityEE" : {
						"LeftArm" : velEE_left,
						"RightArm" : velEE_right
					},
					"KlamptCommandPos" : klampt_command_pos,
					"KlamptSensedPos" : klampt_sensor_pos
				}
				# print('states updated with success!')
			except Exception as e:
				print(e)
			################
			self.server['TRINA_TIME'] = time.time()

			elapsedTime = time.time() - loopStartTime
			if elapsedTime < self.dt:
				time.sleep(self.dt-elapsedTime)
			else:
				pass
		print('\n\n\n\nstopped updating state!!! \n\n\n\n')

	def commandReciever(self,robot,active_modules):
		self.trina_queue_reader = TrinaQueueReader()
		self.dt = 0.0001
		self.robot = robot
		self.interface = RedisInterface(host="localhost")
		self.interface.initialize()
		self.server = KeyValueStore(self.interface)
		self.active_modules = active_modules
		self.init_time = time.time()
		self.loop_counter = 0
		self.command_logger = CommandLogger('EXECUTED_COMMANDS')
		while(True):
			self.loop_counter +=1
			for i in self.always_active:
				self.active_modules[i] = True
			loopStartTime = time.time()
			self.robot_command = self.server['ROBOT_COMMAND'].read()
			if(len(self.empty_command.keys()) != len(self.robot_command.keys())):
				print(self.empty_command.keys(),self.robot_command.keys())
				print('updating list of modules')
				empty_command = {}
				for key in self.robot_command.keys():
					empty_command.update({str(key):[]})
					try:
						self.active_modules[str(key)]
					except Exception as e:
						if(str(key) not in self.always_active):
							self.active_modules[str(key)] = False
						else:
							self.active_modules[str(key)] = True
				self.empty_command = empty_command
			# self.server['ROBOT_COMMAND'] = self.empty_command

			for i in self.robot_command.keys():
				robot_command = self.trina_queue_reader.read(str(i))
				# print(robot_command)
				if (robot_command != []):
					if(self.active_modules[str(i)]):
						commandList = robot_command
						for command in commandList:
							self.run(command)
							# print(command)
							self.command_logger.log_command(command,time.time())
					else:
						print('ignoring commands from {} because it is inactive'.format(str(i)),robot_command)
			elapsedTime = time.time() - loopStartTime	# helper func
			if((time.time()-self.init_time) > 5):
				all_loops_time = time.time() - self.init_time
				self.init_time = time.time()
				print('\nLoop Execution Frequency = {} \n'.format(self.loop_counter/all_loops_time))
				self.loop_counter = 0

			# print('\n\n Frequency of execution loop:', 1/elapsedTime,'\n\n')
			if elapsedTime < self.dt:
				time.sleep(self.dt-elapsedTime)
			else:
				pass

	def run(self,command):
		try:
			exec(command)
		except Exception as e:
			print('there was an error executing your command!',e)
		finally:
			print("command recieved was " + command)

			pass


	#0 -> dead
	#1 -> healthy
	def moduleMonitor(self):
		self.monitoring_dt = 1
		self.tolerance = 10000000000000000
		while not self.shut_down_flag:
			to_restart = []

			loopStartTime = time.time()
			for module in self.modules_dict.keys():
				moduleStatus = self.server["HEALTH_LOG"][module].read()
				if ((time.time()-moduleStatus[1]) > self.tolerance*self.monitoring_dt):
					print("Module " + module + " is dead  due to timeout, queueing restart")
					to_restart.append(module)
				else:
					processes = self.modules_dict[module]
					for pcess in processes:
						if(not(pcess.is_alive())):
							print("Module " + module + " is dead due to dead process, queueing restart")
							to_restart.append(module)
							break
			if(to_restart != []):
				print('restarting modules ' + str(to_restart))
				self.start_modules(to_restart)

			elapsedTime = time.time() - loopStartTime
			if elapsedTime < self.monitoring_dt:
				time.sleep(self.monitoring_dt-elapsedTime)
			else:
				pass


	def sigint_handler(self, signum, frame):
		""" Catch Ctrl+C tp shutdown the robot

		"""
		assert(signum == signal.SIGINT)
		print("SIGINT caught...shutting down the api!")

	def shutdown(self):
		#send shutdown to all modules
		self.shut_down_flag = True
		return 0

	def setRobotToDefault(self):
		leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
		rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
		print('right_Untucked',rightUntuckedConfig)
		if('left_limb' in self.components):
			self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,2)
		if('right_limb' in self.components):
			self.robot.setRightLimbPositionLinear(rightUntuckedConfig,2)

	def start_redis(self):
		print('starting redis')
		origWD = os.getcwd() # remember our original working directory
		#setting up the start of the redis server
		redis_server_path = os.path.expanduser('~/database-server/redis-5.0.4/src/redis-server')
		redis_conf_path = os.path.expanduser('~/database-server/redis.conf')
		redis_folder = os.path.expanduser('~/database-server')
		command_string = '{} {}'.format(redis_server_path,redis_conf_path)
		os.chdir(redis_folder)
		args = shlex.split(command_string)

		if(sys.version_info[0] < 3):
			pid=os.fork()
			if pid == 0:
				try:
					os.setsid()
				except:
					print('could not separate the process.')
				# if pid==0: # new process
				self.redis_pipe = subprocess.Popen(args)
				while(True):
					time.sleep(1000)
		else:

			self.redis_pipe = subprocess.Popen(args,start_new_session = True)

		# reverting back to trina directory
		os.chdir(origWD)

	def start_ros_stuff(self):
		print('starting ros stuff')
		origWD = os.getcwd() # remember our original working directory
		#setting up the start of the redis server
		# catkin_folder = os.path.expanduser('~/catkin_ws/devel/')
		# os.chdir(catkin_folder)
		# os.system('./setup.sh')

		command_string = 'roscore'
		gmapping_string = 'rosrun gmapping slam_gmapping scan:=base_scan _xmax:=10 _xmin:=-10 _ymax:=10 _ymin:=-10'
		ros_args = shlex.split(command_string)
		gmapping_args = shlex.split(gmapping_string)
		if(not rosgraph.is_master_online()):
			self.ros_process = subprocess.Popen(ros_args)
		else:
			print('roscore already running, skipping this part')
		time.sleep(3)

		self.gmapping = subprocess.Popen(gmapping_args)
		print('executed gmapping')
		os.chdir(origWD)

class TrinaQueue(object):
	def __init__(self,key, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
	def push(self,item):
		self.r.rpush(self.key,item)

class TrinaQueueReader(object):
	def __init__(self, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
	def read(self,key):
		with self.r.pipeline() as pipe:
			times = self.r.llen(key)
			for i in range(times):
				pipe.lpop(key)
			res = pipe.execute()
		return res


class CommandLogger(object):
	def __init__(self,key, host = 'localhost', port = 6379,max_length = 10000):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
		self.max_length = max_length
	def log_command(self,command,time):
		self.length = self.r.llen(self.key)
		if(self.length >= self.max_length):
			print('\n\n\n\n\nQUEUE OVERFLOW!!!! \n\n\n\n\n SOMETHING WRONG WITH THE LOGGER?')
		else:
			self.r.rpush(self.key,str([command,time]))


if __name__=="__main__":
	import argparse

	parser = argparse.ArgumentParser(description='Initialization parameters for TRINA')

	server = CommandServer(mode = 'Kinematic',components =  ['right_limb','left_limb','left_gripper'], modules = ['C1','C2','DirectTeleOperation'])
	# server = CommandServer(mode = 'Physical',components =  ['base','left_limb','right_limb','left_gripper'], modules = ['C1','C2','DirectTeleOperation'])
	while(True):
		time.sleep(100)
		pass
