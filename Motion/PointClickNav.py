import os
import signal
import sys
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

import rospy
import sensor_msgs

from datetime import datetime
from motion import Motion
#python files
from global_planner import *
from local_planner import *
from motion_primitives import *
from motion_profile import *
from utils import *
from geometry import *


import Jarvis
#import sensor module
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore

class PointClickNav:
	def __init__(self,debugging = True):
		#if true, run a test locally, otherwise, communicate with Jarvis to get statesssssss
		self.debugging = debugging
		self.last_timestamp = 0.0
		self.state = 'idle' #states are " idle, active"
		self.infoLoop_rate = 0.02
		self.newConfirmation = False
		self.can_send_gridmap = False
		self.exit_flag = False
		if self.debugging:
			self.visualization = True
			self.simulated_robot = Motion(mode = 'Kinematic', codename="anthrax") #the world file has been modified to add a 
			#laser sensor
			self.simulated_robot.startup()
			self.world = self.simulated_robot.getWorld()
			#add some obstacles
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [2, 2, 0]), "test")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-2, 2, 0]), "test1")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [2, -2, 0]), "test2")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-2, -2, 0]), "test3")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 5, 0]), "test")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [5, 0, 0]), "test1")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, -5, 0]), "test2")
			add_terrain(self.world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-5, 0, 0]), "test3")
			vis.add("world",self.world)
			self.curr_pose = self.simulated_robot.sensedBasePosition()
			self.sim = klampt.Simulator(self.world)
			self.lidar = self.sim.controller(0).sensor("lidar")
			self.system_start_time = time.time()
			vis.add("lidar",self.lidar)
			if self.visualization:
				vis.show()
		else:
			#TODO
			self.Jarvis = Jarvis()
			#get current pose
			self.interface = RedisInterface()
			self.interface.initialize()
			self.server = KeyValueStore(self.interface)
			#start sensor_module
			#start visualizer
			self.curr_pose = self.server["ROBOT_STATE"]["Position"]["base"]

		#goal for navigation from user
		self.newRay = False
		self.ray = []

		#start the process that sends laser scans to gmapping
		self.ros_parent_conn, self.ros_child_conn = Pipe()
		self.gmapping_proc = Process(target=self._publishGmappingStuff, args=(self.ros_child_conn, ))
		self.gmapping_proc.start()

		#start the parent ros node
		rospy.init_node("sensing_test_parent")


		self.grid = None	
		while self.grid == None:
			self.lidar.kinematicSimulate(self.world,0.001)
			ros_msg = ros.to_SensorMsg(self.lidar, frame="/base_scan")
			self.ros_parent_conn.send([ros_msg, self.curr_pose,False]) 
			#grid and the gridmap info
			self.grid = get_occupancy_grid("dynamic_map")
			

		self.res = self.grid.info.resolution
		self.radius = 0.5588/2/self.res * 2.0 #radius in terms of grids
		self.gridmap = build_2d_map(self.grid)
		self.preprocessed_gridmap = preprocess(self.gridmap, self.radius)

		#current start and end 
		#transforms world coordinates into the grid (indeces)
		self.start = intify(transform_coordinates((self.curr_pose[0], self.curr_pose[1]), self.grid))
		#wait for user to give goal
		self.end = []

		self._sharedLock = RLock()
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl-c)
	
		#global path computation processes
		#this only start the process but need to be activated to start computing global path
		self.global_path_parent_conn, self.global_path_child_conn = Pipe()
		self.global_path_proc = Process(target=self._computeGlobalPath, args=(self.global_path_child_conn,self.end,self.radius,False))
		self.global_path_proc.start()
		
		main_thread = threading.Thread(target = self._mainLoop)
		state_thread = threading.Thread(target = self._infoLoop)
		state_thread.start()
		main_thread.start()


	def sigint_handler(self, signum, frame):
		""" Catch Ctrl+C tp shutdown the api,
			there are bugs with using sigint_handler.. not used rn.
	
		"""
		assert(signum == signal.SIGINT)
		#logger.warning('SIGINT caught...shutting down the api!')
		print("SIGINT caught...shutting down the api!")
		#self._sharedLock.acquire()
		#self.exit_flag = True
		#self._shutdown()
		#self._sharedLock.release()
		#print(self.end)
		vis.kill()
		self.global_path_parent_conn.send([[],[],[],True,True])
		self.ros_parent_conn.send([[],[],True]) 


	def _infoLoop(self):
		if self.debugging:
			send_ray_once = False	
		while not self.exit_flag:
			loop_start_time = time.time()
			#check if there is new information
			if self.debugging:
				self._sharedLock.acquire()
				if not send_ray_once and (time.time() - self.system_start_time > 8.0):
					self.state = 'active'
					self.newRay = True
					send_ray_once =True
					print('infoLoop: new ray set for main thread')
				if self.state == 'waiting':
					self.newConfirmation = True
					print('infoLoop: plan confirmed')	
				#update current position
				self.curr_pose = self.simulated_robot.sensedBasePosition()
				self.curr_vel = self.simulated_robot.sensedBaseVelocity()
				#send current laser signal...
				self.lidar.kinematicSimulate(self.world,0.001)
				ros_msg = ros.to_SensorMsg(self.lidar, frame="/base_scan")
				self.ros_parent_conn.send([ros_msg, self.curr_pose,False])
				#print('lidar scan sent') 
				self._sharedLock.release()
			else:
				state = self.Jarvis.receiveState()
				self._sharedLock.acquire()
				#TODO: check if new ray or newConfirmation has arrived
				# self.newRay = False
				# self.newConfirmation = False 
				# self.ray = []
				#send new information to Jarvis
				self.Jarvis.send(self.state)
				self.Jarvis.send(self.last_timestamp)

				self._sharedLock.release()
				######send new laser scan#####
				#get lidar from Jarvis
				lidar = state.lidar
				ros_msg = ros.to_SensorMsg(lidar, frame="/base_scan")
				#get base position from Jarvis
				self.curr_pose = state.base_state.measuredPos #robot.base_state.measuredPos
				self.ros_parent_conn.send([ros_msg,curr_pose,False]) 



			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.infoLoop_rate:
				time.sleep(self.infoLoop_rate)
			else:
				time.sleep(0.001) #sleep a small amount of time to avoid occupying the lock
		print('----------')
		print('PointClickNav: state thread exited')


	def _mainLoop(self):
		planning_request_sent = True
		if self.debugging:
			pose_history = []

		
		while not self.exit_flag:
			loop_start_time = time.time()
			self._sharedLock.acquire()
			self.last_timestamp = time.time()
			if self.state == 'idle':
				self._sharedLock.release()
				print("_mainLoop: idling")
			else:
				#receive UI state
				if self.newRay:
					self.newRay = False
					self.state = 'planning'
					planning_request_sent = False
					print("_mainLoop: newRay received")

				if self.newConfirmation:
					self.newConfirmation = False
					self._sharedLock.acquire()
					self.state = 'executing'

					self.global_path_parent_conn.send((self.gridmap,self.start,self.end,True,False))

					#The current disc of the robot...
					self.curr_point = Circle(self.start[::-1], self.radius)
					self.curr_theta = self.curr_pose[2]
					self._sharedLock.release()
					at_end = False
					print("_mainloop: confirmation received and start executing")
				self._sharedLock.release()

				if self.state == 'planning':
					if not planning_request_sent:
						#this will give an initial plan based on the limited 2D map
						#calculate the end position
						if self.debugging:
							self.end = intify(transform_coordinates((4,8), self.grid))
							self.end_theta = 0
						else:
							#TODO Calculate the goal postion and orientation from user
							self.end = self.ray
							self.end_theta
						#get the most recent map
						new_grid = get_occupancy_grid("dynamic_map", timeout=0.001)
						if new_grid is not None:
							self.grid = new_grid
							self.gridmap = build_2d_map(self.grid)
							self.start = intify(transform_coordinates((self.curr_pose[0], self.curr_pose[1]), self.grid))
							self.global_path_parent_conn.send((self.gridmap, self.start,self.end,True,False))
							planning_request_sent = True
						else:
							print("no map found..")
							break

						print("_mainloop: first planning request sent")
					else:
						#check if path planning has finished 
						if self.global_path_parent_conn.poll():
							new_global_path = self.global_path_parent_conn.recv()
							#stop planning and waiting for user confirmation
							self.global_path_parent_conn.send((None,self.start,self.end,False,False))			
							self.global_path = new_global_path
							#This means that the process is ready to accept a new planning request
							can_send_gridmap = True
							end_v = 0.5
							##send path for user to check
							self._sharedLock.acquire()
							if self.debugging:
								pass
							else:
								#TODO
								#send the path to user to confirm
								self.Jarvis
							self.state = 'waiting'
							self._sharedLock.release()
							print("_mainLoop: waiting for confirmation")

				if self.state == 'waiting':
					pass

				if self.state == 'executing':
					print("_mainLoop: executing")
					#####compute local action and send to robot
					#check collision
					collision = self.curr_point.collides(self.gridmap)
					if collision:
						print("collided...this should not have happened")
						break

					#check if the global path his been completed
					dist_to_goal = l2_dist(self.curr_point.center, self.end)
					if dist_to_goal < 0.2 / self.res or at_end:
						#if complete, let user know it is complete
						self._sharedLock.acquire()
						if self.debugging:
							print('at end')
						else:
							#TODO
							self.Jarvis.send
						self.state = 'idle'
						#stop calculating global path
						self.global_path_parent_conn.send((None,self.start,self.end,False,False))	
						self._sharedLock.release()

				    # near goal, run a special controller?
					if dist_to_goal < 1.0 / self.res:
						at_end = True
						primitives = [LocalPath([(self.curr_point.center[0], self.curr_point.center[1], self.curr_theta), (self.end[0], self.end[1], self.end_theta)])]
						end_v = 0
					else:
						primitives = get_primitives(self.curr_point.center, self.curr_theta, self.radius*2, self.radius*1)

					#find the primitive that leads to the global path
					closest = evaluate_primitives(self.curr_point, primitives, self.global_path, self.gridmap)
					kglobal = klampt.model.trajectory.Trajectory(milestones = [transform_back([x, y], self.grid) for x, y in zip(self.global_path.get_xs(), self.global_path.get_ys())])

					if self.debugging:
						if self.visualization:
							vis.add("kglobaltraj", kglobal)
							if len(pose_history) > 0:
								khistory_traj = klampt.model.trajectory.Trajectory(milestones = [p for p in pose_history])
								vis.add("khistorytraj", khistory_traj)
								vis.setColor("khistorytraj", 0, 255, 0)

					#sometimes might need to rotate in place.. rotate in place is not one of the primitives
					if closest is None:
						if self.debugging:
							print("No prim!!!")
							self.simualted_robot.setBaseVelocity([0.0, 0.4])
							new_pose = self.curr_pose
							new_pose = transform_coordinates(new_pose, self.grid)

							self.curr_point = Circle((snew_pose[0], new_pose[1]), self.radius)
							self.curr_theta = new_pose[2]
							continue
						else:
							#TODO
							pass

					#Get the primitve milestones		
					xs, ys, thetas = closest.get_xytheta(20)
					#proceed to finish one primitive
					acc = 0.5
					max_v = 0.5
					#generate the velocity profile
					profile = generate_profile(closest, acc/self.res, max_v/self.res, 0.01, end_v = end_v/self.res, start_v=self.curr_vel[0]/self.res)
					end_v = profile[-1].v
					milestones = [transform_back([x, y], self.grid) for x, y in zip(xs, ys)]
					ktraj = klampt.model.trajectory.Trajectory(milestones = milestones)

					N = len(profile)
					time_thresh = 0.2 #time for the 20 milestons of primitive to complete
					start_time = time.time()

					for i in range(N):
						iteration_start = time.time()
						state = profile[i]

						if not at_end and iteration_start - start_time > time_thresh:
							end_v = min(max_v, end_v)
							break

						self._sharedLock.acquire()
						if self.debugging:
							pose_history.append([self.curr_pose[0], self.curr_pose[1]])
						pose = transform_coordinates(self.curr_pose, self.grid)
						self._sharedLock.release()

						curr_target = (state.x, state.y)
						trans_target = so2.apply(-pose[2], vectorops.sub(curr_target, (pose[0], pose[1])))

						linear_error = trans_target[0]
						cross_track_error = trans_target[1]

						#tune the gains here?
						k_linear = 1.0 * self.res 
						k_angular = 1.0 * self.res

						vel = [state.v * self.res + k_linear*linear_error, state.w + k_angular*cross_track_error]
						if self.debugging:
							if self. visualization:
								vis.add("klocaltraj", ktraj)
								vis.setColor("klocaltraj", 0, 0, 255)
							self.simulated_robot.setBaseVelocity(vel)
						else:
							#TODO
							#jarvis set velocity for the base
							pass

						new_pose = deepcopy(self.curr_pose)
						new_pose = transform_coordinates(new_pose, self.grid)
						curr_theta = new_pose[2]

						elapsed_time = time.time() - iteration_start
						remaining_time = max(0.01 - elapsed_time, 0.0)
						time.sleep(remaining_time)

					#after the primitive is completed, update its posit
					#In the next iteration, a new primitive will be generated given current pose..
					new_pose = deepcopy(self.curr_pose)
					new_pose = transform_coordinates(new_pose, self.grid)
					self.curr_point = Circle((new_pose[0], new_pose[1]), self.radius)
					self.curr_theta = new_pose[2]


					##### receive new map and replan global path
					new_grid = get_occupancy_grid("dynamic_map", timeout=0.001)
					if can_send_gridmap and new_grid is not None:
						self.grid = new_grid
						self.gridmap = build_2d_map(self.grid)
						self.start = intify(transform_coordinates((self.curr_pose[0], self.curr_pose[1]), self.grid))
						self.global_path_parent_conn.send((self.gridmap, self.start,self.end,True,False))
						can_send_gridmap = False
					else:
						print("no map found..")

					if self.global_path_parent_conn.poll():
						new_global_path = self.global_path_parent_conn.recv()
						self.global_path_parent_conn.send((None,self.start,self.end,False,False))			
						self.global_path = new_global_path
						can_send_gridmap = True

			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.infoLoop_rate:
				time.sleep(self.infoLoop_rate)
			else:
				time.sleep(0.001) #sleep a small amount of time to avoid occupying all the computation resource
		print('----------')
		print('PointClickNav: main thread exited')

	def activate(self):
		pass


	def _publishGmappingStuff(self,conn):
		rospy.init_node("sensing_test_child")
		pub = rospy.Publisher("base_scan", sensor_msgs.msg.LaserScan)

		ros_msg = None
		curr_pose_child = None
		exit = False
		while not exit:
			if conn.poll():
				ros_msg, curr_pose_child, exit = conn.recv()
				if exit:
					break
			if ros_msg is not None and curr_pose_child is not None:
				pub.publish(ros_msg)
				self._publishTf(curr_pose_child) 
		print('----------')
		print('PointClickNav: publish gmapping path exited')
	def _computeGlobalPath(self,conn, end, radius,active):
		gridmap = None
		exit = False
		while not exit:
			if conn.poll():
				gridmap, start,end,active,exit= conn.recv()
				if exit:
					break

			if active:
				if gridmap is not None:
					preprocessed_gridmap = preprocess(gridmap, radius)
					dists, parents = navigation_function(preprocessed_gridmap, end, radius)
					global_path = get_path(parents, start, end)
					conn.send(global_path)
					gridmap = None
			else:
				time.sleep(0.01)
		print('----------')
		print('PointClickNav: compute global path exited')

	def _publishTf(self,curr_pose):
		x, y, theta = curr_pose
		theta = theta % (math.pi*2)
		br = tf.TransformBroadcaster()
		br.sendTransform([x, y, 0], tf.transformations.quaternion_from_euler(0, 0, theta), rospy.Time.now(), "base_link", "odom")
		br.sendTransform([0.2, 0, 0.2], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_scan", "base_link")

	def _shutdown(self):
		self._sharedLock.acquire()
		self.exit_flag = True
		#print("shutdown:",self.exit_flag,id(self.exit_flag[0]),id(self))
		self._sharedLock.release()
		return

if __name__=="__main__":
	demo = PointClickNav(debugging = True)
	#time.sleep(10)
	#print('\n\n\n\n\n\n calling shutdown, boys! \n\n\n\n\n\n\n')
	#demo._shutdown()