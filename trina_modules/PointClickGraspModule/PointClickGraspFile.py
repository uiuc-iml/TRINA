import os, sys
import signal
import time
import math
from threading import Thread, Lock, RLock
import threading
import numpy as np
from multiprocessing import Process, Pipe
import logging
from copy import deepcopy, copy
from klampt.math import vectorops, so3
from klampt.model import ik, collide
from klampt import WorldModel, vis
from TRINAConfig import *
import rospy
import sensor_msgs
# import pbd
from datetime import datetime
from motion import Motion
# python files
from .global_planner import *
from .local_planner import *
from .motion_primitives import *
from motion_profile import *
from .utils import *
from .geometry import *
from matplotlib import pyplot as plt

from sensor_msgs.msg import LaserScan

from klampt import WorldModel, Geometry3D
from klampt import vis
from klampt.math import so3, se3
import trimesh
import os
import random
import numpy as np
import math
import cv2

model_path = '../Resources/shared_data/objects/'
mesh_model_path = '../Resources/grasping/models/'


def convertMsg(klampt_sensor, frame, stamp="now"):
    measurements = klampt_sensor.getMeasurements()
    res = LaserScan()
    if stamp == 'now':
        stamp = rospy.Time.now()
    elif isinstance(stamp, (int, float)):
        stamp = rospy.Time(stamp)
    res.header.frame_id = frame
    res.header.stamp = stamp
    res.angle_max = float(klampt_sensor.getSetting("xSweepMagnitude"))
    res.angle_min = -1.0 * res.angle_max
    measurement_count = float(klampt_sensor.getSetting("measurementCount"))
    res.angle_increment = (res.angle_max - res.angle_min) / measurement_count
    res.time_increment = float(klampt_sensor.getSetting("xSweepPeriod"))
    res.range_min = float(klampt_sensor.getSetting("depthMinimum"))
    res.range_max = float(klampt_sensor.getSetting("depthMaximum"))
    res.ranges = measurements
    res.intensities = []
    return res


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


class PointClickGrasp:
    def __init__(self, Jarvis=None, debugging=False, mode='Kinematic'):
        # if true, run a test locally, otherwise, communicate with Jarvis to get state
        print("PointCLickGrasp starting")
        self.debugging = debugging
        self.last_timestamp = 0.0
        self.state = 'idle'  # states are used internally, there are idle, active, planning etc
        self.status = 'idle'  # idle: state == idle; active: state: active/planning/executing etc
        self.infoLoop_rate = 0.02
        self.can_send_gridmap = False
        self.exit_flag = False
        self.mode = mode
        self.visualization = False
        print(mode, debugging)
        time.sleep(10)
        if self.debugging:

            self.simulated_robot = Motion(mode='Kinematic',
                                          codename="anthrax")  # the world file has been modified to add a
            # laser sensor
            self.simulated_robot.startup()
            self.world = self.simulated_robot.getWorld()
            # add some obstacles
            builder = testingWorldBuilder(30, 30, world=self.world)
            self.world = builder.getWorld()

            # self.world.saveFile('data/TRINA_world_anthrax_pointClick.xml')

            exit()
            vis.add("world", self.world)
            self.curr_pose = self.simulated_robot.sensedBasePosition()
            self.sim = klampt.Simulator(self.world)
            self.lidar = self.sim.controller(0).sensor("lidar")
            self.system_start_time = time.time()
            vis.add("lidar", self.lidar)
            if self.visualization:
                vis.show()
        else:
            self.jarvis = Jarvis
            base_q = self.jarvis.sensedBasePosition()
            self.curr_pose = base_q

            if self.visualization:
                # start visualizer
                self.vis_world = WorldModel()
                self.vis_world.readFile('data/TRINA_world_anthrax_PointClick.xml')
                self.vis_robot = self.vis_world.robot(0)
                self.vis_robot.setConfig(get_klampt_model_q('anthrax', base=base_q))
                vis.add("world", self.vis_world)

            if self.mode == 'Kinematic':
                if self.visualization:
                    vis.add("lidar", self.lidar)
                    vis.show()
            elif self.mode == 'Physical':
                pass
            # TODO

            self.system_start_time = time.time()
        # goal for navigation from user
        self.terminate_command = False  # flag for if user commanded a termination
        self.ray1 = ([], [])  # source and direction
        self.ray2 = ([], [])

        # start the parent ros node
        rospy.init_node("sensing_test_parent")
        # get the first grid
        self.grid = None

        # Debugging.,.
        # while self.grid == None:
        # 	#grid and the gridmap info
        # 	self.grid = get_occupancy_grid("dynamic_map")
        # 	print("\n\n\n It seems that we cannot get a grid from gmapping, it there something wrong? - Changing for Yifan\n\n\n\n")
        # 	time.sleep(0.5)
        self.grid = get_occupancy_grid("dynamic_map")

        self.res = self.grid.info.resolution
        self.radius = 0.5588 / 2 / self.res * 2.0  # radius in terms of grids
        self.gridmap = build_2d_map(self.grid).T
        self.preprocessed_gridmap = preprocess(self.gridmap, self.radius)

        # current start and end
        # transforms world coordinates into the grid (indeces)
        self.start = intify(transform_coordinates((self.curr_pose[0], self.curr_pose[1]), self.grid))
        # wait for user to give goal
        self.end = []
        self._sharedLock = RLock()
        signal.signal(signal.SIGINT, self.sigint_handler)  # catch SIGINT (ctrl-c)

        # global path computation processes
        # this only start the process but need to be activated to start computing global path
        self.global_path_parent_conn, self.global_path_child_conn = Pipe()
        self.global_path_proc = Process(target=self._computeGlobalPath,
                                        args=(self.global_path_child_conn, self.end, self.radius, False))
        self.global_path_proc.start()

        main_thread = threading.Thread(target=self._mainLoop)
        state_thread = threading.Thread(target=self._infoLoop)
        state_thread.start()
        main_thread.start()

    def sigint_handler(self, signum, frame):
        """ Catch Ctrl+C tp shutdown the api,
            there are bugs with using sigint_handler.. not used rn.

        """
        assert (signum == signal.SIGINT)
        # logger.warning('SIGINT caught...shutting down the api!')
        print("SIGINT caught...shutting down the api!")
        vis.kill()
        self.global_path_parent_conn.send([[], [], [], True, True])

    # self.ros_parent_conn.send([[],[],True])

    def return_threads(self):
        return [self._infoLoop, self._mainLoop]

    def return_processes(self):
        return [self.global_path_proc]

    def _infoLoop(self):
        if self.debugging:
            send_ray_once = False
        while not self.exit_flag:
            self.jarvis.log_health()
            loop_start_time = time.time()
            # check if there is new information
            if self.debugging:
                self._sharedLock.acquire()
                if not send_ray_once and (time.time() - self.system_start_time > 8.0):
                    self.state = 'active'
                    self.newRay = True
                    send_ray_once = True
                    print('infoLoop: new ray set for main thread')
                if self.state == 'waiting':
                    self.newConfirmation = True
                    print('infoLoop: plan confirmed')
                # update current position
                self.curr_pose = self.simulated_robot.sensedBasePosition()

                # send current laser signal...
                self.lidar.kinematicSimulate(self.world, 0.001)
                ros_msg = ros.to_SensorMsg(self.lidar, frame="/base_scan")
                self.ros_parent_conn.send([ros_msg, self.curr_pose, False])
                # print('lidar scan sent')
                self._sharedLock.release()
            else:
                # left_q = self.jarvis.sensedLeftLimbPosition()
                # right_q = self.jarvis.sensedRightLimbPosition()
                base_q = self.jarvis.sensedBasePosition()
                self.curr_vel = self.jarvis.sensedBaseVelocity()
                if (self.visualization):
                    self.vis_robot.setConfig(get_klampt_model_q('anthrax', base=base_q))
                status = self.jarvis.getActivityStatus()

                # TODO get terminate flag question
                # terminate_flag = self.jarvis.get

                # self._sharedLock.acquire()

                if (status == 'active'):
                    if (self.status == 'idle'):
                        print('\n\n\n\n starting up Autonomous Navigation Module! \n\n\n\n\n')
                        self.activate()
                        leftUntuckedConfig = [-0.2028, -2.1063, -1.610, 3.7165, -0.9622, 0.0974]
                        rightUntuckedConfig = self.jarvis.mirror_arm_config(leftUntuckedConfig)
                        self.jarvis.setLeftLimbPositionLinear(leftUntuckedConfig, 2)
                        self.jarvis.setRightLimbPositionLinear(rightUntuckedConfig, 2)
                        print("entering main loop")
                        self.jarvis.setBaseVelocity([0.1, 0])
                        time.sleep(2)
                        self.jarvis.setBaseVelocity([0, 0])
                elif (status == 'idle'):
                    if self.status == 'active':
                        self.deactivate()

                # if terminate_flag:
                # self.terminate_command = True

                self._sharedLock.acquire()
                self.curr_pose = base_q
                self._sharedLock.release()
            if self.state == "active":
                self.rgbdimage = self.jarvis.get_rgbd_images()
                depth_right = self.rgbdimage['realsense_right'][1]
                #plt.imshow(depth_right)
                #plt.show()
                #plt.imshow(self.rgbdimage['realsense_right'][0])
                #plt.show()

            elapsed_time = time.time() - loop_start_time
            if elapsed_time < self.infoLoop_rate:
                time.sleep(self.infoLoop_rate)
            else:
                time.sleep(0.001)  # sleep a small amount of time to avoid occupying the lock
        print('----------')
        print('PointClickNav: state thread exited')

    def _mainLoop(self):
        self.global_path = None
        self.curr_point = None
        planning_request_sent = True
        pose_history = []

        

        while not self.exit_flag:
            # print('fromPointClick:',self.state)
            loop_start_time = time.time()
            self.last_timestamp = time.time()
            if self.terminate_command:
                self._sharedLock.acquire()
                self.state = 'idle'
                self.jarvis.setBaseVelocity([0, 0])
                self.terminate_command = False  # flag for if user commanded a termination
                self.global_path = None
                self.end = None
                self.start = None
                self.end_theta = None
                self.global_path_parent_conn.send((self.gridmap, self.start, self.end, False, False))
                self._sharedLock.release()

            if self.state == 'idle':
                # print("_mainLoop: idling")
                pass
            elif self.state == 'active':
                print('_mainLoop:active')
                # ask for a ray
                self.ray = self.jarvis.sendAndGetRayClickUI()
                self.rgbdimage = self.jarvis.get_rgbd_images()
                depth_right = self.rgbdimage['realsense_right'][1]
                print(self.rgbdimage['realsense_right'][0])
                plt.imshow(self.rgbdimage['realsense_right'][0])
                #cv2.waitKey(0)
                plt.show()
                #plt.imshow(self.rgbdimage['realsense_right'][0])
                #plt.show()
                self._sharedLock.acquire()
                self.state = 'planning'
                planning_request_sent = False
                print("_mainLoop: newRay received")
                self._sharedLock.release()


            elif self.state == 'planning':
                if not planning_request_sent:
                    print('_mainLoop:planning')
                    # this will give an initial plan based on the limited 2D map
                    # calculate the end position
                    if self.debugging:
                        self.end = intify(transform_coordinates((4, 8), self.grid))
                        self.end_theta = 0
                    else:
                        end = self._groundIntersect(
                            (self.ray['FIRST_RAY']['source'], self.ray['FIRST_RAY']['destination']))
                        direction = vectorops.sub(self._groundIntersect(
                            (self.ray['SECOND_RAY']['source'], self.ray['SECOND_RAY']['destination'])), end)
                    # get the most recent map and send the request
                    new_grid = get_occupancy_grid("dynamic_map", timeout=0.001)
                    if new_grid is not None:  # this should always find a map if things go right
                        self.grid = new_grid
                        self.gridmap = build_2d_map(self.grid).T
                        self.end = intify(transform_coordinates((end[0], end[1]), self.grid))
                        self.end_theta = math.atan2(direction[1], direction[0])
                        self.start = intify(transform_coordinates((self.curr_pose[0], self.curr_pose[1]), self.grid))
                        self.global_path_parent_conn.send((self.gridmap, self.start, self.end, True, False))
                        planning_request_sent = True

                    else:
                        print("no map found by gmapping during planning...")
                        self.jarvis.sendConfirmationUI('Error', 'A gmapping error has occurred....Returning to idle')
                        self.state = 'idle'

                # if the planning request is already sent to process, then just check if planning has finished
                else:
                    # check if path planning has finished
                    if self.global_path_parent_conn.poll():
                        new_global_path = self.global_path_parent_conn.recv()
                        if new_global_path == None:
                            self.state = 'idle'
                            planning_request_sent = False
                            self.jarvis.sendConfirmationUI('Error',
                                                           'A global path does not seem to be possible....Returning to idle')
                            continue

                        # stop planning and waiting for user confirmation
                        self.global_path_parent_conn.send((None, self.start, self.end, False, False))
                        self.global_path = new_global_path

                        # This means that the process is ready to accept a new planning request
                        self.can_send_gridmap = True
                        end_v = 0.5
                        if not self.debugging:
                            print('sending the trajectory.....')
                            self.jarvis.sendTrajectoryUI(klampt.model.trajectory.Trajectory(
                                milestones=[transform_back([x, y], self.grid) + [0.05] for x, y in
                                            zip(self.global_path.get_xs(), self.global_path.get_ys())]))
                            time.sleep(3)
                            ans = self.jarvis.sendAndGetConfirmationUI('Request', 'Please Confirm the Trajectory')
                            if ans == 'YES':
                                self.state = 'executing'
                                self.jarvis.sendConfirmationUI('info', 'Path has started executing')
                                # unpause the planning process, #actually I don't think this is needed
                                # self.global_path_parent_conn.send((self.gridmap,self.start,self.end,True,False))
                                # The current disc of the robot. Used for planning
                                self.curr_point = Circle(self.start[::-1], self.radius)
                                self.curr_theta = self.curr_pose[2]
                                at_end = False
                                print("_mainloop: confirmation received and start executing")
                                ##for testing
                                self.execution_start_time = time.time()
                            else:
                                self.deactivate()
                        # TODO implemented a way to replan

            # elif self.state == 'waiting':
            # 	#if the user has agreed to the trajectory
            # 	if self.new_confirmation:
            # 		self.new_confirmation = False
            # 		self._sharedLock.acquire()
            # 		self.state = 'executing'
            # 		#unpause the planning process
            # 		self.global_path_parent_conn.send((self.gridmap,self.start,self.end,True,False))
            # 		#The current disc of the robot. Used for planning
            # 		self.curr_point = Circle(self.start[::-1], self.radius)
            # 		self.curr_theta = self.curr_pose[2]
            # 		self._sharedLock.release()
            # 		#variable used for local planning
            # 		at_end = False
            # 		print("_mainloop: confirmation received and start executing")

            elif self.state == 'executing':
                print("_mainLoop: executing")
                #####compute local action and send to robot
                # check collision
                collision = self.curr_point.collides(self.gridmap.T)
                if collision:
                    print("collided...this should not have happened")
                    break

                # check if the global path his been completed
                dist_to_goal = l2_dist(self.curr_point.center, self.end)
                if dist_to_goal < 0.2 / self.res or at_end:
                    self._sharedLock.acquire()
                    if self.debugging:
                        print('at end')
                    else:
                        # self.jarvis.sendConfirmationUI('info','Path has finished')
                        self.jarvis.setBaseVelocity([0, 0])
                        self.jarvis.changeActivityStatus(['UI'], ['PointClickNav'])
                        print('execution has completed')
                    self.state = 'idle'
                    self.global_path = None
                    self.end = None
                    self.start = None
                    self.end_theta = None
                    # stop calculating global path
                    self.global_path_parent_conn.send((None, self.start, self.end, False, False))
                    self._sharedLock.release()
                    self.jarvis.changeActivityStatus(['UI'], ['PointClickNav'])

                # near goal, run a special controller?
                if dist_to_goal < 1.0 / self.res:
                    at_end = True
                    primitives = [LocalPath([(self.curr_point.center[0], self.curr_point.center[1], self.curr_theta),
                                             (self.end[0], self.end[1], self.end_theta)])]
                    end_v = 0
                else:
                    primitives = get_primitives(self.curr_point.center, self.curr_theta, self.radius * 2,
                                                self.radius * 1)

                # find the primitive that leads to the global path
                closest = evaluate_primitives(self.curr_point, primitives, self.global_path, self.gridmap.T)
                kglobal = klampt.model.trajectory.Trajectory(milestones=[transform_back([x, y], self.grid) for x, y in
                                                                         zip(self.global_path.get_xs(),
                                                                             self.global_path.get_ys())])

                if self.visualization:
                    vis.add("kglobaltraj", kglobal)
                    if len(pose_history) > 0:
                        khistory_traj = klampt.model.trajectory.Trajectory(milestones=[p for p in pose_history])
                        vis.add("khistorytraj", khistory_traj)
                        vis.setColor("khistorytraj", 0, 255, 0)

                # sometimes might need to rotate in place.. rotate in place is not one of the primitives
                if closest is None:
                    if self.debugging:
                        print("No prim!!!")
                        self.simualted_robot.setBaseVelocity([0.0, 0.4])
                        new_pose = self.curr_pose
                        new_pose = transform_coordinates(new_pose, self.grid)

                        self.curr_point = Circle((new_pose[0], new_pose[1]), self.radius)
                        self.curr_theta = new_pose[2]
                        continue
                    else:
                        print("No prim!!!")
                        self.jarvis.setBaseVelocity([0.0, 0.4])
                        time.sleep(0.1)
                        self._sharedLock.acquire()
                        new_pose = self.curr_pose
                        new_pose = transform_coordinates(new_pose, self.grid)

                        self.curr_point = Circle((new_pose[0], new_pose[1]), self.radius)
                        self.curr_theta = new_pose[2]
                        self._sharedLock.release()
                        continue

                # Get the primitve milestones
                xs, ys, thetas = closest.get_xytheta(20)
                # proceed to finish one primitive
                acc = 0.3
                max_v = 0.3
                # generate the velocity profile
                start_time = time.time()
                profile = generate_profile(closest, acc / self.res, max_v / self.res, 0.01, end_v=end_v / self.res,
                                           start_v=self.curr_vel[0] / self.res)
                print('generating primitive took:', time.time() - start_time)
                end_v = profile[-1].v
                milestones = [transform_back([x, y], self.grid) for x, y in zip(xs, ys)]
                ktraj = klampt.model.trajectory.Trajectory(milestones=milestones)

                N = len(profile)
                time_thresh = 0.2  # time for the 20 milestons of primitive to complete
                start_time = time.time()

                for i in range(N):
                    # check current status
                    if self.state == 'idle':
                        self.jarvis.setBaseVelocity([0, 0])
                        break

                    start = time.time()
                    if self.terminate_command:
                        self._sharedLock.acquire()
                        self.state = 'idle'
                        self.jarvis.setBaseVelocity([0, 0])
                        self.global_path = None
                        self.end = None
                        self.start = None
                        self._sharedLock.release()
                        break
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

                    # tune the gains here?
                    k_linear = 0.5 * self.res  # 1.0
                    k_angular = 0.5 * self.res  # 1.0

                    vel = [state.v * self.res + k_linear * linear_error, state.w + k_angular * cross_track_error]
                    if self.visualization:
                        vis.add("klocaltraj", ktraj)
                        vis.setColor("klocaltraj", 0, 0, 255)
                    if self.debugging:
                        self.simulated_robot.setBaseVelocity(vel)
                    else:
                        self.jarvis.setBaseVelocity(vel)

                    new_pose = deepcopy(self.curr_pose)
                    new_pose = transform_coordinates(new_pose, self.grid)
                    self.curr_theta = new_pose[2]

                    elapsed_time = time.time() - iteration_start
                    remaining_time = max(0.01 - elapsed_time, 0.0)
                    time.sleep(remaining_time)
                    print('\n\n loop execution frequency PointClickNav', 1 / (time.time() - start), '\n\n')
                # after the primitive is completed, update its posit
                # In the next iteration, a new primitive will be generated given current pose..
                self._sharedLock.acquire()
                new_pose = deepcopy(self.curr_pose)
                new_pose = transform_coordinates(new_pose, self.grid)
                self.curr_point = Circle((new_pose[0], new_pose[1]), self.radius)
                self.curr_theta = new_pose[2]
                self._sharedLock.release()

                # check status
                if self.state == 'idle':
                    self.jarvis.setBaseVelocity([0, 0])
                    continue

                ##### receive new map and replan global path
                new_grid = get_occupancy_grid("dynamic_map", timeout=0.001)
                if self.can_send_gridmap and new_grid is not None:
                    self.grid = new_grid
                    self.gridmap = build_2d_map(self.grid).T
                    self.start = intify(transform_coordinates((self.curr_pose[0], self.curr_pose[1]), self.grid))

                    start_time = time.time()
                    self.global_path_parent_conn.send((self.gridmap, self.start, self.end, True, False))
                    # print(self.start,self.end)
                    print('elapsed time:', time.time() - start_time)
                    self.can_send_gridmap = False

                if new_grid is None:
                    print("no map found from gmapping..")

                if self.global_path_parent_conn.poll():
                    new_global_path = self.global_path_parent_conn.recv()
                    if new_global_path:
                        self.global_path = new_global_path
                        self.jarvis.sendTrajectoryUI(klampt.model.trajectory.Trajectory(
                            milestones=[transform_back([x, y], self.grid) + [0.05] for x, y in
                                        zip(self.global_path.get_xs(), self.global_path.get_ys())]))
                    else:
                        print("new global path is empty")
                        self.jarvis.sendConfirmationUI('Error',
                                                       'A global path does not seem to be possible....Returning to idle')
                        self.jarvis.setBaseVelocity([0, 0])
                        self.state = 'idle'
                        continue
                    self.can_send_gridmap = True

            elapsed_time = time.time() - loop_start_time
            if elapsed_time < self.infoLoop_rate:
                time.sleep(self.infoLoop_rate)
            else:
                time.sleep(0.001)  # sleep a small amount of time to avoid occupying all the computation resource
        print('----------')
        print('PointClickNav: main thread exited')

    def _computeGlobalPath(self, conn, end, radius, active):
        gridmap = None
        exit = False
        while not exit:
            if conn.poll():
                gridmap, start, end, active, exit = conn.recv()
                if exit:
                    break
                # makes more sense that you only compute path when a request is sent
                if active:
                    if gridmap is not None:
                        preprocessed_gridmap = preprocess(gridmap, radius)
                        dists, parents = navigation_function(preprocessed_gridmap, end, radius)
                        global_path = get_path(parents, start, end)
                        conn.send(global_path)
                        gridmap = None
                    time.sleep(0.001)
                else:
                    time.sleep(0.01)
        print('----------')
        print('PointClickNav: compute global path exited')

    def _publishTf(self, curr_pose):
        x, y, theta = curr_pose
        theta = theta % (math.pi * 2)
        br = tf.TransformBroadcaster()
        br.sendTransform([x, y, 0], tf.transformations.quaternion_from_euler(0, 0, theta), rospy.Time.now(),
                         "base_link", "odom")
        br.sendTransform([0.2, 0, 0.2], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),
                         "base_scan", "base_link")

    def _shutdown(self):
        self._sharedLock.acquire()
        self.exit_flag = True
        # print("shutdown:",self.exit_flag,id(self.exit_flag[0]),id(self))
        self._sharedLock.release()
        if self.visualization:
            vis.kill()
        return

    def _groundIntersect(self, ray):
        s = ray[0]
        d = ray[1]
        # des = ray[1] #destination not direction
        # d = vectorops.unit(vectorops.sub(des,s)) #direction
        assert d[2] != 0.0
        alpha = -s[2] / d[2]

        return [s[0] + alpha * d[0], s[1] + alpha * d[1]]

    def activate(self):
        self._sharedLock.acquire()
        self.state = 'active'
        self.status = 'active'
        self._sharedLock.release()

    def deactivate(self):
        self._sharedLock.acquire()
        self.state = 'idle'
        self.status = 'idle'
        self.jarvis.setBaseVelocity([0, 0])
        self.confirm_request_sent = False
        self.global_path = None
        self.end = None
        self.start = None
        self.end_theta = None
        self.terminate_command = False  # flag for if user commanded a termination
        self.global_path_parent_conn.send(([], [], [], False, False))
        self._sharedLock.release()


if __name__ == "__main__":
    demo = PointClickGrasp(debugging=False)
# time.sleep(10)
# print('\n\n\n\n\n\n calling shutdown, boys! \n\n\n\n\n\n\n')
# demo._shutdown()
