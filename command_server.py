import time,math,datetime
import threading
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
import sys
import os
import klampt
from klampt import vis
from klampt import io
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates,config,trajectory,collide
from klampt.math import vectorops,so3,se3
from klampt.vis import GLSimulationPlugin
from klampt.math import se3
from klampt import vis, Geometry3D
from klampt.model import sensing
from OpenGL.GLUT import *
from threading import Thread
from klampt import sim
import random
import trimesh
import rosgraph
from Motion import MotionClient
if(sys.version_info[0] < 3):
	from future import *
else:
	from importlib import reload
from Jarvis import Jarvis
import redis
import traceback
import logging

robot_ip = 'http://localhost:8080'

ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

model_path = './Resources/shared_data/objects/'
mesh_model_path = './Resources/grasping/models/'

class testingWorldBuilder():
	def __init__(self, floor_length=20, floor_width=20, world=[]):
		if (world == []):
			self.w = WorldModel()
		else:
			self.w = world
		self.floor = Geometry3D()
		self.floor.loadFile(model_path + "cube.off")
		self.floor.transform([floor_length, 0, 0, 0, floor_width, 0, 0, 0, 0.01],
							 [-floor_length / 2.0, -floor_width / 2.0, -0.01])
		floor_terrain = self.w.makeTerrain("floor")
		floor_terrain.geometry().set(self.floor)
		floor_terrain.appearance().setColor(0.4, 0.3, 0.2, 1.0)

		###colors
		self.light_blue = [3.0 / 255.0, 140.0 / 255.0, 252.0 / 255.0, 1.0]
		self.wall_green = [50.0 / 255.0, 168.0 / 255.0, 143.0 / 255.0, 1.0]
		###sizes
		self.table_length = 1.2
		self.table_width = 0.8
		self.table_top_thickness = 0.03
		self.table_height = 0.8
		self.leg_width = 0.05
		self.cube_width = 0.05

	def getWorld(self):
		return self.w

	def addTableTopScenario(self, x=0, y=0):
		"""
		add a table with objects on top, the center of table can be set
		Parameters:
		--------------
		x,y: floats, the position of the table center
		"""

		self.addTable(x, y)
		# add some cubes
		self.addCube((so3.from_axis_angle(([0, 0, 1], 0.5)), [x, y - 0.7, self.table_height]), self.cube_width,
					 [1.0, 0, 0, 1], 1)
		# add one mesh
		random.seed(30)
		self.addRandomMesh([0 + x, -1.0 + y, self.table_height], 1)
		self.addRandomMesh([0 + x, -1.2 + y, self.table_height], 2)
		self.addRandomMesh([0.2 + x, -1.0 + y, self.table_height], 3)
		self.addRandomMesh([-0.2 + x, -1.2 + y, self.table_height], 4)

	def addIndoorNavScenario(self):
		"""
		Add 4 rooms and a table
		"""

		wall_thickness = 0.2
		room_size = [8.0, 6.0, 4.0]
		self.addRoom(room_size, wall_thickness, T=([1, 0, 0, 0, 1, 0, 0, 0, 1], [-6, -6, 0]), ID=1)
		self.addRoom(room_size, wall_thickness, T=([1, 0, 0, 0, 1, 0, 0, 0, 1], [6, -6, 0]), ID=2)
		self.addRoom(room_size, wall_thickness, T=(so3.from_axis_angle(([0, 0, 1], math.pi / 1.0)), [6, 6, 0]), ID=3)
		self.addRoom(room_size, wall_thickness, T=(so3.from_axis_angle(([0, 0, 1], math.pi / 1.0)), [-6, 6, 0]), ID=4)
		self.addTable(-6, -6)
		return

	def addRoom(self, room_size=[10.0, 8.0, 4.0], wall_thickness=0.2, T=([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 0]), ID=1):
		"""
		Add a room to the world
		Parameters:
		-------------------
		room_size = list of 3 floats, the width,depth, and height of the room
		wall_thickess = float
		T:rigid transform of the room
		ID: ID of the room
		"""

		# left wall
		self._addTerrain(model_path + "cube.off",
						 se3.mul(T, ([wall_thickness, 0, 0, 0, room_size[1] - wall_thickness, 0, 0, 0, room_size[2]], \
									 [-room_size[0] / 2.0, -room_size[1] / 2.0, 0])), self.wall_green,
						 "wall1_" + str(ID))
		# wall with window
		self._addTerrain(model_path + "cube.off", se3.mul(T, ([room_size[0], 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
															  [-room_size[0] / 2.0, -room_size[1] / 2.0, 0])),
						 self.wall_green, "wall2_" + str(ID))
		self._addTerrain(model_path + "cube.off", se3.mul(T, ([room_size[0], 0, 0, 0, wall_thickness, 0, 0, 0, 2], \
															  [-room_size[0] / 2.0, -room_size[1] / 2.0, 2])),
						 self.wall_green, "wall3_" + str(ID))
		self._addTerrain(model_path + "cube.off",
						 se3.mul(T, ([room_size[0] / 2.0 - 0.5, 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
									 [-room_size[0] / 2.0, -room_size[1] / 2.0, 1])), self.wall_green,
						 "wall4_" + str(ID))
		self._addTerrain(model_path + "cube.off",
						 se3.mul(T, ([room_size[0] / 2.0 - 0.5, 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
									 [0.5, -room_size[1] / 2.0, 1])), self.wall_green, "wall5_" + str(ID))

		# right wall
		self._addTerrain(model_path + "cube.off",
						 se3.mul(T, ([wall_thickness, 0, 0, 0, room_size[1] - wall_thickness, 0, 0, 0, room_size[2]], \
									 [room_size[0] / 2.0 - wall_thickness, -room_size[1] / 2.0, 0])), self.wall_green,
						 "wall6_" + str(ID))

		# the wall with door
		self._addTerrain(model_path + "cube.off",
						 se3.mul(T, ([room_size[0] - 2.5, 0, 0, 0, wall_thickness, 0, 0, 0, room_size[2]], \
									 [-room_size[0] / 2.0, room_size[1] / 2.0 - wall_thickness, 0])), self.wall_green,
						 "wall7_" + str(ID))
		self._addTerrain(model_path + "cube.off", se3.mul(T, ([1, 0, 0, 0, wall_thickness, 0, 0, 0, room_size[2]], \
															  [-room_size[0] / 2.0 + (room_size[0] - 1),
															   room_size[1] / 2.0 - wall_thickness, 0])),
						 self.wall_green, "wall8_" + str(ID))
		self._addTerrain(model_path + "cube.off", se3.mul(T, ([1.5, 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
															  [-room_size[0] / 2.0 + (room_size[0] - 1 - 1.5),
															   room_size[1] / 2.0 - wall_thickness, 3])),
						 self.wall_green, "wall9_" + str(ID))

	##Functions below add individual objects
	def addCube(self, T, side_length, color, ID, object_mass=0.1):
		"""
		Add a cube to the world.
		Parameters:
		--------------
		T:world transform of the cube
		side_length: float, size of the cube
		color: RGBA values, (0-1)
		ID: int, cannot duplicate
		mass:object mass added at the object geometric center
		"""

		self._addRigidObject(model_path + "cube.off",
							 ([side_length, 0, 0, 0, side_length, 0, 0, 0, side_length, ], [0, 0, 0]), T, \
							 color, object_mass, [side_length / 2.0, side_length / 2.0, side_length / 2.0],
							 "cube" + str(ID))

	def addMesh(self, path, T, color, mass, ID):
		"""
		Add a mesh to the world.
		Parameters:
		--------------
		path: path to the mesh file
		T:world transform of the mesh
		color: RGBA values, (0-1)
		mass:object mass added at the object geometric center
		ID: int, cannot duplicate
		"""
		mesh = trimesh.load(path)
		mesh_center = mesh.centroid.tolist()

		# load the rigid object in the world
		self._addRigidObject(path, ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0] * 3), T, \
							 color, mass, mesh_center, "item" + str(ID))

	def addRandomMesh(self, t, ID=1):
		"""
		Add a household item to the world, randonmly selected from the library.
		Color is also determined randomly. Mass set to 1kg arbitrarily
		Parameters:
		--------------
		t:world position of the mesh
		ID: int, cannot duplicate
		"""
		meshpaths = []
		for file in os.listdir(mesh_model_path):
			if file.endswith(".ply"):
				meshpaths.append(os.path.join(mesh_model_path, file))

		meshpath = random.choice(meshpaths)
		mesh = trimesh.load(meshpath)
		mesh_center = mesh.centroid.tolist()
		# Z_min = np.min(mesh.vertices[:,2])

		# t[2] = t[2]+mesh_center[2]-Z_min
		# load the rigid object in the world
		self._addRigidObject(meshpath, ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0] * 3), ([1, 0, 0, 0, 1, 0, 0, 0, 1], t), \
							 (random.random(), random.random(), random.random(), 1.0), 0.1, mesh_center,
							 "item" + str(ID))

	def addTable(self, x=0, y=0):
		"""
		Add a table to the world
		Parameters:
		--------------
		x,y: floats, the x,y position of the center of the table
		"""
		table_top = Geometry3D()
		table_leg_1 = Geometry3D()
		table_leg_2 = Geometry3D()
		table_leg_3 = Geometry3D()
		table_leg_4 = Geometry3D()

		table_top.loadFile(model_path + "cube.off")
		table_leg_1.loadFile(model_path + "cube.off")
		table_leg_2.loadFile(model_path + "cube.off")
		table_leg_3.loadFile(model_path + "cube.off")
		table_leg_4.loadFile(model_path + "cube.off")

		table_top.transform([self.table_length, 0, 0, 0, self.table_width, 0, 0, 0, \
							 self.table_top_thickness], [0, 0, self.table_height - self.table_top_thickness])
		table_leg_1.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height \
							   - self.table_top_thickness], [0, 0, 0])
		table_leg_2.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height - \
							   self.table_top_thickness], [self.table_length - self.leg_width, 0, 0])
		table_leg_3.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height -
							   self.table_top_thickness],
							  [self.table_length - self.leg_width, self.table_width - self.leg_width, 0])
		table_leg_4.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height -
							   self.table_top_thickness], [0, self.table_width - self.leg_width, 0])

		table_geom = Geometry3D()
		table_geom.setGroup()
		for i, elem in enumerate([table_top, table_leg_1, table_leg_2, table_leg_3, table_leg_4]):
			g = Geometry3D(elem)
			table_geom.setElement(i, g)
		table_geom.transform([0, -1, 0, 1, 0, 0, 0, 0, 1], [x - self.table_length / 2.0, y - self.table_width / 2.0, 0])
		table = self.w.makeTerrain("table")
		table.geometry().set(table_geom)
		table.appearance().setColor(self.light_blue[0], self.light_blue[1], self.light_blue[2], self.light_blue[3])

	def addRobot(self, path, T):
		"""
		Add a robot to the world. You can directly use Klampt functions to add to the world as well
		Parameters:
		------------
		path: path to the robot model
		T: transform of the base of the robot
		"""

		world.loadElement(path)
		item = world.rigidObject(0)
		item.setTransform([1, 0, 0, 0, 1, 0, 0, 0, 1],
						  [ee_pos[0] - mesh_center[0], ee_pos[1] - mesh_center[1], -Zmin + 0.02])

	def _addRigidObject(self, path, T_g, T_p, color, object_mass, Com, name):
		item_1_geom = Geometry3D()
		item_1_geom.loadFile(path)
		item_1_geom.transform(T_g[0], T_g[1])
		item_1 = self.w.makeRigidObject(name)
		item_1.geometry().set(item_1_geom)
		item_1.appearance().setColor(color[0], color[1], color[2], color[3])
		bmass = item_1.getMass()
		bmass.setMass(object_mass)
		bmass.setCom(Com)
		item_1.setMass(bmass)
		item_1.setTransform(T_p[0], T_p[1])
		return item_1

	def _addTerrain(self, path, T, color, name):
		item_1_geom = Geometry3D()
		item_1_geom.loadFile(path)
		item_1_geom.transform(T[0], T[1])
		item_1 = self.w.makeTerrain(name)
		item_1.geometry().set(item_1_geom)
		item_1.appearance().setColor(color[0], color[1], color[2], color[3])
		return item_1

class CommandServer:

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

		self.codename = codename

		self.start_ros_stuff()
		self.world_file = world_file
		# we then proceed with startup as normal
		self.always_active = set(['UI','devel','debug','DirectTeleoperation'])
		self.interface = RedisInterface(host="localhost")
		self.interface.initialize()
		self.server = KeyValueStore(self.interface)
		self.logger = logging.getLogger('reem')
		self.logger.setLevel(logging.ERROR)
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
			builder = testingWorldBuilder(30,30,world = self.world)
			builder.addTableTopScenario(x = 1.5,y = 1.0)
			self.world = builder.getWorld()
			self.sensor_module = Camera_Robot(robot = self.robot,world = self.world)
			print('\n\n\n\n\n initialization of Kinematic sensor module sucessfull!!\n\n\n')

			time.sleep(3)
		if(self.mode == 'Physical'):
			self.sensor_module = None #Camera_Robot(robot = self.robot, mode = self.mode, cameras=['realsense_right', 'realsense_left'])
			print('\n\n\n\n\n initialization of Physical sensor module sucessfull!!\n\n\n')
			time.sleep(5)

		self.health_dict = {}
		# create the list of threads
		self.modules_dict = {}

		manager = Manager()
		print('\nstarting all the modules\n')
		self.active_modules = manager.dict()
		for i in self.always_active:
			self.active_modules[i] = True
		self.start_modules(self.modules,startup = True)
		time.sleep(3)

		print('\nall modules started succesfully\n')
		print('\n starting state receiver \n')
		stateRecieverThread = threading.Thread(target=self.stateReciever)
		stateRecieverThread.start()		
		print('\n state receiver started!\n')
		print('\n starting pause/resume check\n')
		pauseResumeThread = threading.Thread(target=self.pauseResumeChecker)
		pauseResumeThread.start()	
		print('\n starting command receiver \n')
		commandRecieverThread = Process(target=self.commandReciever, args=(self.robot, self.active_modules))
		commandRecieverThread.daemon = True
		commandRecieverThread.start()
		print('\n command receiver started!\n')
		print('\n starting module monitor \n')
		moduleMonitorThread = threading.Thread(target=self.moduleMonitor)
		moduleMonitorThread.start()
		print('\n module monitor started\n')

		atexit.register(self.shutdown_all)
		if mode == "Kinematic":
			self.setRobotToDefault()
		
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
		global_EEWrench_left = None
		local_EEWrench_left = None
		global_EEWrench_right = None
		local_EEWrench_right = None
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
		klampt_q = get_klampt_model_q(self.codename,left_limb = self.query_robot.sensedLeftLimbPosition(), right_limb = self.query_robot.sensedRightLimbPosition(), base = pos_base)
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
	
	def pauseResumeChecker(self):
		while not self.shut_down_flag:
			time.sleep(0.5)
			try:
				pause0 = self.server["UI_STATE"]["UIlogicState"]["stop"].read()
				pause1 = self.server["Phone_Stop"].read()
				if pause0 == False and pause1 == False:
					self.robot.resumeMotion()
				else:
					self.robot.pauseMotion()
			except Exception as e:
				traceback.print_exc()

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
		while not self.shut_down_flag:
			loopStartTime = time.time()
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

				klampt_q = get_klampt_model_q(self.codename,left_limb = pos_left, right_limb = pos_right, base = pos_base)
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
				traceback.print_exc()
			################
			self.server['TRINA_TIME'] = time.time()

			elapsedTime = time.time() - loopStartTime
			if elapsedTime < self.dt:
				time.sleep(self.dt - elapsedTime)
			else:
				time.sleep(1e-6)
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
				# print("Command in command_server", robot_command)
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
				time.sleep(self.dt - elapsedTime)
			else:
				time.sleep(1e-6)
	
	def run(self,command):
		try:
			exec(command)
		except Exception as e:
			tb = traceback.format_exc()
			print('there was an error executing your command!',tb)
		finally:
			pass
			# print("command recieved was " + str(command))


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
				time.sleep(self.monitoring_dt - elapsedTime)
			else:
				time.sleep(1e-6)


	def sigint_handler(self, signum, frame):
		""" Catch Ctrl+C tp shutdown the robot

		"""
		assert(signum == signal.SIGINT)
		print("SIGINT caught...shutting down the api!")
		self.shutdown()

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
			# print('QUEUE OVERFLOW!!!! SOMETHING WRONG WITH THE LOGGER?')
			#print('\n\n\n\n\nQUEUE OVERFLOW!!!! \n\n\n\n\n SOMETHING WRONG WITH THE LOGGER?')
			pass
		else:
			self.r.rpush(self.key,str([command,time]))


if __name__=="__main__":
	import argparse

	parser = argparse.ArgumentParser(description='Initialization parameters for TRINA')

	server = CommandServer(mode = 'Physical',components =  ['left_limb','right_limb','base'], modules = ['DirectTeleOperation'], codename = 'bubonic')
	# server = CommandServer(mode = 'Kinematic',components =  ['left_limb','right_limb'], modules = ['C1','C2','DirectTeleOperation','PointClickNav', 'PointClickGrasp'], codename = 'bubonic')
	
	print(server.robot.closeLeftRobotiqGripper())
	print(server.robot.sensedLeftEETransform())
	while(True):
		time.sleep(100)
		pass
