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
from klampt.math import vectorops, so3, se3
from klampt.model import ik, collide
from klampt import WorldModel, Geometry3D, vis
import TRINAConfig
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
    def __init__(self, Jarvis=None, debugging=False,):
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
        self.visualization = False
        
        time.sleep(10)
        if self.debugging:
            pass
        else:
            base_q = self.jarvis.sensedBasePosition()
            self.curr_pose = base_q
            
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
                    self.vis_robot.setConfig(TRINAConfig.get_klampt_model_q('anthrax', base=base_q))
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
                        
                        if self.jarvis.robot.mode() == "Kinematic":
                            leftUntuckedConfig = trina_settings.left_arm_config('untucked')
                            rightUntuckedConfig = self.jarvis.mirror_arm_config(leftUntuckedConfig)
                            self.jarvis.setLeftLimbPositionLinear(leftUntuckedConfig, 2)
                            self.jarvis.setRightLimbPositionLinear(rightUntuckedConfig, 2)
                            
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
                self.ray = self.jarvis.sendAndGetRayClickUI()
                self.rgbdimage = self.jarvis.get_rgbd_images()
                depth_left = self.rgbdimage['realsense_left'][1]
                color_left = self.rgbdimage['realsense_left'][0]
                color_left = np.array(color_left)[:,:,:]
                
                #cv2.imshow("image", np.array(depth_left))
                #cv2.waitKey(0)
                #im = seg.segmentation.blur_frame(np.array(color_left))
                imgray = rgb2gray(color_left)
                imscaled = (imgray*256).astype("uint16")
                segmented, _ = seg.mixture_model(imscaled, debug=True)
                #labels = seg.watershed(imgray, segmented)
                #print(labels)
                mask_overlay = label2rgb(segmented,np.array(color_left),colors=[(255,0,0),(0,0,255), (0, 255,0), (255,255,0),(0,255,255), (255, 0,255)],alpha=0.01, bg_label=0, bg_color=None)
                cv2.imshow("image", np.array(mask_overlay))
                cv2.waitKey(0)
                
                color = o3d.geometry.Image(np.array(color_left))
                depth = o3d.geometry.Image(np.array(depth_left)[:,:])
                
                rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth); 
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
                    rgbd_image, o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
                )
                # Flip it, otherwise the pointcloud will be upside down
                pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
                
                o3d.io.write_point_cloud("test_grasping.pcd", pcd)
                
                
                
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
                        #self.jarvis.setBaseVelocity([0, 0])
                        #self.jarvis.changeActivityStatus(['UI'], ['PointClickNav'])
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
                        #self.simualted_robot.setBaseVelocity([0.0, 0.4])
                        new_pose = self.curr_pose
                        new_pose = transform_coordinates(new_pose, self.grid)

                        self.curr_point = Circle((new_pose[0], new_pose[1]), self.radius)
                        self.curr_theta = new_pose[2]
                        continue
                    else:
                        print("No prim!!!")
                        #self.jarvis.setBaseVelocity([0.0, 0.4])
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
                        #self.jarvis.setBaseVelocity([0, 0])
                        break

                    start = time.time()
                    if self.terminate_command:
                        self._sharedLock.acquire()
                        self.state = 'idle'
                        #self.jarvis.setBaseVelocity([0, 0])
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

                    new_pose = deepcopy(self.curr_pose)
                    new_pose = transform_coordinates(new_pose, self.grid)
                    self.curr_theta = new_pose[2]

                    elapsed_time = time.time() - iteration_start
                    remaining_time = max(0.01 - elapsed_time, 0.0)
                    time.sleep(remaining_time)
                    print('\n\n loop execution frequency PointClickGrasp', 1 / (time.time() - start), '\n\n')
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
                    #self.jarvis.setBaseVelocity([0, 0])
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
                        #self.jarvis.setBaseVelocity([0, 0])
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

