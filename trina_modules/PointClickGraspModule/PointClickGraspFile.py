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
import sensor_msgs
from datetime import datetime

# python files
from .global_planner import *
from .local_planner import *
from .motion_primitives import *
from motion_profile import *
from .utils import *
from .geometry import *

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
import segmentation as seg
import open3d as o3d
from skimage.color import rgb2gray, label2rgb
from skimage import color

class PointClickGrasp:
    def __init__(self, Jarvis=None, debugging=False, mode='Kinematic'):
        print("PointCLickGrasp starting")
        self.system_start_time = time.time()
        self.jarvis = Jarvis
        self.debugging = debugging
        self.last_timestamp = 0.0
        self.state = 'idle'  # states are used internally, there are idle, active, planning etc
        self.status = 'idle'  # idle: state == idle; active: state: active/planning/executing etc
        self.infoLoop_rate = 0.02
        self.can_send_gridmap = False
        self.exit_flag = False
        self.mode = mode
        self.visualization = False
        
        time.sleep(10)
        if self.debugging:
            pass
        else:
            base_q = self.jarvis.sensedBasePosition()
            self.curr_pose = base_q

            if self.mode == 'Kinematic':
                if self.visualization:
                    vis.add("lidar", self.lidar)
                    vis.show()
            elif self.mode == 'Physical':
                pass

            
        # goal for navigation from user
        self.terminate_command = False  # flag for if user commanded a termination
        self.ray1 = ([], [])  # source and direction
        self.ray2 = ([], [])

        self._sharedLock = RLock()
        signal.signal(signal.SIGINT, self.sigint_handler)  # catch SIGINT (ctrl-c)

        main_thread = threading.Thread(target=self._mainLoop)
        state_thread = threading.Thread(target=self._infoLoop)
        state_thread.start()
        main_thread.start()

    def sigint_handler(self, signum, frame):
        """ Catch Ctrl+C tp shutdown the api,
            there are bugs with using sigint_handler.. not used rn.

        """
        assert (signum == signal.SIGINT)
        print("SIGINT caught...shutting down the api!")

    def return_threads(self):
        return [self._infoLoop, self._mainLoop]

    def return_processes(self):
        return []

    def _infoLoop(self):
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
                        print('\n\n\n\n starting up Point Click Grasp Module! \n\n\n\n\n')
                        self.activate()
                        print("entering main loop")
                        print(os.getcwd())
                        
                        if self.mode == "Kinematic":
                            leftUntuckedConfig = [-0.2028, -2.1063, -1.610, 3.8165, -0.9622, 1.0974]
                            rightUntuckedConfig = self.jarvis.mirror_arm_config(leftUntuckedConfig)
                            self.jarvis.setLeftLimbPositionLinear(leftUntuckedConfig, 2)
                            self.jarvis.setRightLimbPositionLinear(rightUntuckedConfig, 2)
                            
                            self.jarvis.setBaseVelocity([0.15, 0])
                            time.sleep(2.2)
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
                #print("Printing rgbd image")
                #print(np.array(self.rgbdimage['realsense_right'][0]))
                #depth_right = self.rgbdimage['realsense_right'][0]
                #cv2.imshow("image", np.array(depth_right))
                #cv2.waitKey(0)

            elapsed_time = time.time() - loop_start_time
            if elapsed_time < self.infoLoop_rate:
                time.sleep(self.infoLoop_rate)
            else:
                time.sleep(0.001)  # sleep a small amount of time to avoid occupying the lock
        print('----------')
        print('PointClickGrasp: state thread exited')

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
                #self.ray = self.jarvis.sendAndGetRayClickUI()
                
                leftUntuckedConfig = [-0.1028, -2.5063, -1.710, 3.7165, -0.9622, 1.0974]
                rightUntuckedConfig = self.jarvis.mirror_arm_config(leftUntuckedConfig)
                self.jarvis.setLeftLimbPositionLinear(leftUntuckedConfig, 2)
                self.jarvis.setRightLimbPositionLinear(rightUntuckedConfig, 2)
                time.sleep(4)
                
                
                self.rgbdimage = self.jarvis.get_rgbd_images()
                depth_left = self.rgbdimage['realsense_left'][1]
                color_left = self.rgbdimage['realsense_left'][0]
                depth_right = self.rgbdimage['realsense_right'][1]
                color_right = self.rgbdimage['realsense_right'][0]
                color_left = np.array(color_left)[:,:,:]
                color_right = np.array(color_right)[:,:,:]
                
                #cv2.imshow("image", np.array(depth_left))
                #cv2.waitKey(0)
                #im = seg.segmentation.blur_frame(np.array(color_left))
                imgray = rgb2gray(color_right)
                imscaled = (imgray*256).astype("uint16")
                segmented, _ = seg.mixture_model(imscaled, debug=True)
                #labels = seg.watershed(imgray, segmented)
                #print(labels)
                mask_overlay = label2rgb(segmented,np.array(color_left),colors=[(255,0,0),(0,0,255), (0, 255,0), (255,255,0),(0,255,255), (255, 0,255)],alpha=0.01, bg_label=0, bg_color=None)
                
                
                #cv2.imshow("image", color_left)
                #cv2.waitKey(0)
                #if cv2.waitKey(1) & 0xFF == ord('q'):
                #    break
                
                klampt_to_o3d = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0],[0,0,0,1]])
                inv_k_to_o3d = np.linalg.inv(klampt_to_o3d)
                
                color = o3d.geometry.Image(np.array(color_left))
                #print(list(depth_left))
                depth_left_flipped = np.array(depth_left).astype(np.float32)*(1)
                depth = o3d.geometry.Image(np.array(depth_left_flipped))
                # Flip it, otherwise the pointcloud will be upside down
                
                self.pcds = self.jarvis.get_point_clouds()
                print(self.pcds)
                pcd = self.pcds['realsense_left']
                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, 1.0, convert_rgb_to_intensity=False)
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
                pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                print(pcd.has_points())
                o3d.io.write_point_cloud("test_grasping.pcd", pcd)
                
                
                """
                self._sharedLock.acquire()
                self.state = 'planning'
                planning_request_sent = False
                print("_mainLoop: newRay received")
                self._sharedLock.release()
                """
                
                os.system("/home/motion/gpd/build/detect_grasps /home/motion/gpd/cfg/eigen_params.cfg /home/motion/TRINA/test_grasping.pcd")
                
                leftUntuckedConfig = [-0.2528, -3.4263, -0.510, 3.7165, -0.9622, 0.9974]
                self.jarvis.setLeftLimbPositionLinear(leftUntuckedConfig, 2)
                time.sleep(3)
                
                self.state = 'idle'

            elapsed_time = time.time() - loop_start_time
            if elapsed_time < self.infoLoop_rate:
                time.sleep(self.infoLoop_rate)
            else:
                time.sleep(0.001)  # sleep a small amount of time to avoid occupying all the computation resource
        print('----------')
        print('PointClickGrasp: main thread exited')

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
        #self.jarvis.setBaseVelocity([0, 0])
        self.confirm_request_sent = False
        self.global_path = None
        self.end = None
        self.start = None
        self.end_theta = None
        self.terminate_command = False  # flag for if user commanded a termination
        self._sharedLock.release()


if __name__ == "__main__":
    demo = PointClickGrasp(debugging=False)

