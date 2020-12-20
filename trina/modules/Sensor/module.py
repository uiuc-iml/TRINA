# This is a quality of life improvement module to enable students to use one of TRINA's arms equipped with an intel realsense cameras for grasping tasks
# UIUC - IML - Feburary 2020
# Author: Joao Marcos Marques - jmc12@illinois.equipped
# This code must be run within the TRINA folder.

import atexit
import numpy as np
import open3d as o3d
import pickle
import klampt
import numpy
import time
from klampt.model import sensing
from threading import Thread
import threading
import copy
from OpenGL.GLUT import *
from OpenGL.GL import *
from klampt.math import vectorops,so3,se3
from klampt import WorldModel, vis
import klampt.io.numpy_convert
import sys
import os
try:
    import trina
except ImportError:
    sys.path.append(os.path.expanduser("~/TRINA"))
    import trina
from trina import jarvis
from trina.sensors import ZedCamera,RealSenseCamera

try:
    import tf
    import rospy
    import sensor_msgs
    from sensor_msgs.msg import LaserScan
    _have_ros = True
except ImportError:
    _have_ros = False

from multiprocessing import Process, Pipe

from .api import *

class CameraData:
    def __init__(self,settings=None):
        if settings is not None:
            self.serial_number = settings["serial_number"]
            self.link = settings["link"]
            calib_fn = settings["transform"]
            trina_dir = trina.setup.root_dir()
            calib_fn = os.path.join(trina_dir,calib_fn )
            transform = np.load(open(calib_fn, 'rb'))
            # we then create the inverse transform
            self.local_transform = klampt.io.numpy_convert.from_numpy(transform,'RigidTransform')
        else:
            self.serial_number = None
            self.link = None
            self.local_transform = None

        self.driver = None
        self.want_images = False
        self.want_point_cloud = False
        self.images = None
        self.point_cloud = None
        self.world_point_cloud = None
        self.world_transform = None

class SensorModule(jarvis.APIModule):
    """
    This is the primary class of this helper module. It instantiates a motion client and handles the streamed inputs from the realsense camera
    """

    def __init__(self, Jarvis, cameras= None, mode='Kinematic', ros_active = True):
        jarvis.APIModule.__init__(self,Jarvis)

        # we first check if the parameters are valid:
        # checking if cameras make sense:
        self.cameras = cameras
        # we now verify if the camera configuration file makes sense
        # we then try to connect to the motion_client (we always use the same arm - and we never enable the base for now)
        self.mode = mode
        self.active_cameras = {}
        self.update_lock = threading.Lock()
        self.requests = []    #a list of pending requests
        self.ros_active = ros_active
        global have_ros
        if not _have_ros:
            if self.ros_active:
                print('\n\n\n\n')
                print("ROS support requested, but rospy could not be imported")
                print('\n\n\n\n')
            self.ros_active = False

        if self.mode == 'Physical':
            self.world = trina.setup.robot_model_load()
            self.temprobot = self.world.robot(0)
            camera_settings = trina.settings.camera_settings()
            if cameras is None:
                cameras = list(self.cameras.keys())
            for camera in self.cameras:
                if camera not in camera_settings:
                    raise ValueError('invalid camera %s selected. Valid cameras are %s (see TRINA/Settings/).\nPlease update camera selection and try again'%(camera,', '.join(camera_settings.keys())))
                # if the camera is valid, import and configure the camera
                self.active_cameras[camera] = CameraData(camera_settings[camera])
                self.startSimpleThread(self.update_camera,args=(camera,),dt=camera_settings[camera].get('framerate',1.0/30.0),initfunc=self.init_camera,name=camera,dolock=False)
        elif self.mode == 'Kinematic':
            self.world = trina.setup.robot_model_load()
            self.temprobot = self.world.robot(0)
            self.simworld = trina.setup.simulation_world_load()
            self.simrobot = self.simworld.robot(0)
            self.sim = trina.setup.set_robot_sensor_calibration(self.simworld)
            
            #determine which camera sensors are available
            all_cams = []
            i = 0
            while True:
                sensori = self.sim.controller(0).sensor(i)
                if sensori.type() == 'CameraSensor':
                    all_cams.append(sensori.name())
                if sensori.type() == '':  #out of sensors
                    break
                i += 1
            if self.cameras is None:
                self.cameras = all_cams

            for camera in self.cameras:
                simcam = self.sim.controller(0).sensor(camera)
                if len(simcam.name()) == 0: #invalid camera
                    raise ValueError('invalid camera %s selected. Valid cameras are %s (see robot model).\nPlease update camera selection and try again'%(camera,', '.join(all_cams)))
                else:
                    self.active_cameras[camera] = CameraData()
                    self.active_cameras[camera].link = int(simcam.getSetting('link'))
                    self.active_cameras[camera].local_transform = sensing.get_sensor_xform(simcam)
                    self.active_cameras[camera].device = simcam
            self.lidar = self.sim.controller(0).sensor("lidar")
            self.lidar_data = None
            self.lidar_transform = None
            self.system_start = time.time()
            
            self.glutWindowID = None
            self.startSimpleThread(self.update_sim,0.1,initfunc=self.init_GL,name="update_sim",dolock=False)
        self.startSimpleThread(self.update_robot,dt=1.0/30.0,name="update_robot",dolock=False)

    def api(self,*args,**kwargs):
        return SensorAPI(self,"sensors",*args,**kwargs)

    def terminate(self):
        jarvis.APIModule.terminate(self)
        #by now all threads should have terminated, so no risk of problems
        for c in self.active_cameras:
            if hasattr(self.active_cameras[c].device,'safely_close'):
                self.active_cameras[c].device.safely_close()
        self.active_cameras = {}
        if self.mode == 'Kinematic':
            pass
            #There's no way to restart GLUT this way
            #if self.glutWindowID is not None:
            #    glutDestroyWindow(self.glutWindowID)

        self.sim = None
        self.temprobot = None
        self.world = None

    def init_camera(self,camera_name):
        camera = self.active_cameras[camera_name]
        serial_number = camera.serial_number
        try:
            if camera_name.startswith('realsense'):
                driver = RealSenseCamera(serial_number)
            elif camera_name.startswith('zed'):
                driver = ZedCamera(serial_number)
            else:
                print('Verify camera names, no camera match found!')
                raise TypeError('No compatible camera found!')
            camera.driver = driver
            atexit.register(camera.driver.safely_close)
            print('sucessfully initialized the camera! ')
        except Exception as e:
            print('This camera ', camera_name,
                ' is currently unavailable. Verify that it is connected and try again \n\n')
            with self.update_lock:
                del self.active_cameras[camera_name]
            self.stopThread(camera_name)

    def update_camera(self,camera_name):
        camera = self.active_cameras[camera_name]
        if camera.want_images or camera.want_point_cloud:
            camera.driver.update()
            if camera.want_images:
                camera.images = camera.latest_rgbd_images()
            if camera.want_point_cloud:
                camera.point_cloud = camera.latest_point_cloud()
                with self.update_lock:
                    camera.world_transform = self.camera_transform(camera_name,format='numpy')
                    print("World transform",camera.world_transform)
                camera.world_point_cloud =  camera.point_cloud.transform(camera.world_transform)
        with self.update_lock:
            #check if i need to update any pending requests
            newrequests = []
            for req in self.requests:
                promise,query,cameras,results = req
                if cameras is None or camera_name in cameras:
                    if query == 'get_rgbd_images':
                        camera.want_images = True
                        if camera.images is not None:
                            results[camera_name] = camera.images
                    elif query == 'get_point_clouds':
                        camera.want_point_cloud = True
                        if camera.world_point_cloud is not None:
                            results[camera_name] = camera.world_point_cloud
                    ntrigger = len(self.active_cameras) if cameras is None else len(cameras)
                    if len(results) == ntrigger:
                        #fire the callback if all camera data is available
                        promise.callback(results)
                        continue
                newrequests.append(req)
            self.requests = newrequests

    def camera_transform(self, camera_name, format='klampt'):
        #assume update_lock is called?
        camera = self.active_cameras[camera_name]
        link = self.temprobot.link(camera.link)
        T = se3.mul(link.getTransform(),camera.local_transform)
        if format == 'klampt':
            return T
        else:
            return klampt.io.numpy_convert.to_numpy(T,'RigidTransform')

    def get_rgbd_images(self, cameras=None):
        if cameras is None:
            cameras = list(self.active_cameras.keys())
        output = {}
        if type(cameras) == str:
            cameras = [cameras]
        elif type(cameras) != list:
            raise TypeError(
                'Selected cameras must be either a string or a list of strings')
        for camera in cameras:
            if camera not in self.active_cameras:
                continue
            self.active_cameras[camera].want_images = True
            output[camera] = self.active_cameras[camera].images
        return output

    def get_point_clouds(self, cameras=None):
        """
        Returns the point cloud from the referred source in the robot's base frame.

        Args:
            cameras ([str]): A string or list of strings containing which cameras
            we are using for this process.
        Returns:
            output {camera:point_cloud} : A dictionary containing an open3D point
            cloud for each camera string for which a point_cloud is available. 
            point_cloud is None if the requested camera is not yet producing images.
        """
        if cameras is None:
            cameras = list(self.active_cameras.keys())
        if type(cameras) == str:
            cameras = [cameras]
        elif type(cameras) != list:
            raise TypeError(
                'Selected cameras must be either a string or a list of strings')
        output = {}
        for camera in cameras:
            if camera not in self.active_cameras:
                #raise an error here?
                continue
            self.active_cameras[camera].want_point_cloud = True
            local_pc = self.active_cameras[camera].point_cloud
            if local_pc is not None:
                with self.update_lock:
                    assert self.active_cameras[camera].world_transform is not None
                    world_pc = local_pc.transform(self.active_cameras[camera].world_transform)
            else:
                world_pc = None
            output[camera] = world_pc
        return output

    def update_robot(self):
        self.jarvis.log_health()
        try:
            # print(self.jarvis.robot.sensedRobotq(),self.jarvis.robot.sensedRightEETransform(),self.jarvis.robot.sensedLeftEETransform())
            q = self.jarvis.robot.sensedRobotq()
        except Exception as e:
            print('error updating robot state')
            print(e)
            return
        with self.update_lock:
            self.temprobot.setConfig(q)
            
    def init_GL(self):
        # GLEW WORKAROUND
        glutInit([])
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE |
                            GLUT_DEPTH | GLUT_MULTISAMPLE)
        glutInitWindowSize(1, 1)
        self.glutWindowID = glutCreateWindow("Sensor module hidden GLUT window")

        # Default background color
        glClearColor(0.8, 0.8, 0.9, 0)
        # Default light source
        glLightfv(GL_LIGHT0, GL_POSITION, [0, -1, 2, 0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1, 1, 1, 1])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [1, 1, 1, 1])
        glEnable(GL_LIGHT0)

        glLightfv(GL_LIGHT1, GL_POSITION, [-1, 2, 1, 0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.5, 0.5, 0.5, 1])
        glLightfv(GL_LIGHT1, GL_SPECULAR, [0.5, 0.5, 0.5, 1])
        glEnable(GL_LIGHT1)

        if self.ros_active:
            rospy.init_node("SensorModule_publisher")
            self.pub = rospy.Publisher("base_scan", LaserScan)


    def update_sim(self):
        with self.update_lock:
            self.simrobot.setConfig(self.temprobot.getConfig())
        dt = 1.0/30.0 # this is unused in kinematicSimulate
        #assume temprobot is updated
        for k,cam in self.active_cameras.items():
            if cam.want_images or cam.want_point_cloud:
                cam.device.kinematicSimulate(self.simworld, dt)
                images = sensing.camera_to_images(cam.device, image_format='numpy', color_format='channels')
                if cam.want_point_cloud:
                    if int(cam.device.getSetting("depth")) == 0:
                        print("Simulated camera",k,"does not have depth information")
                        pcd = 'error'
                        cam.want_point_cloud = False
                    else:
                        rgb,depth = images
                        xfov = float(cam.device.getSetting('xfov'))
                        yfov = float(cam.device.getSetting('yfov'))
                        local_pc = klampt.model.sensing.image_to_points(depth,rgb,xfov,yfov)
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(local_pc[:, :3])
                        pcd.colors = o3d.utility.Vector3dVector(local_pc[:, 3:])
                    with self.update_lock:
                        world_transform = self.camera_transform(k,format='numpy')
                    world_pcd = camera.point_cloud.transform(world_transform)

                    with self.update_lock:
                        #update everything in one go
                        cam.images = images
                        cam.point_cloud = pcd
                        cam.world_point_cloud = world_pcd
                        cam.world_transform = world_transform
                elif cam.want_images:
                    cam.images = images

        if self.ros_active:
            self.lidar.kinematicSimulate(self.simworld,dt)
            with self.update_lock:
                self.lidar_data = self.lidar.getMeasurements()
                self.lidar_transform = sensing.get_sensor_xform(self.lidar,self.simrobot)
            self._publishTf(curr_pose) 
            #ros_msg = self.convertMsg(self.lidar, frame="/base_scan")
            ros_msg = klampt.io.ros.to_SensorMsg(self.lidar,frame="/base_scan")
            self.pub.publish(ros_msg)
            
        with self.update_lock:
            #check for updates on pending requests
            newrequests = []
            for req in self.requests:
                promise,query,cameras,results = req
                if cameras is None:
                    cameras = list(self.active_cameras.keys())
                if query == 'get_rgbd_images':
                    for name in cameras:
                        camera = self.active_cameras[name]
                        camera.want_images = True
                        if camera.images is not None:
                            results[name] = camera.images
                elif query == 'get_point_clouds':
                    for name in cameras:
                        camera = self.active_cameras[name]
                        camera.want_point_cloud = True
                        if camera.point_cloud is not None:
                            if isinstance(camera.point_cloud,str):
                                #errored out
                                results[name] = camera.point_cloud
                            else:
                                assert camera.world_transform is not None
                                results[name] = camera.point_cloud.transform(camera.world_transform)
                #fire the callback if all camera data is available
                if len(results) == len(cameras):
                    promise.callback(results)
                    continue
                else:
                    newrequests.append(req)
            self.requests = newrequests

    def _publishTf(self,curr_pose):
        x, y, theta = curr_pose
        theta = theta % (math.pi*2)
        br = tf.TransformBroadcaster()
        br.sendTransform([x, y, 0], tf.transformations.quaternion_from_euler(0, 0, theta), rospy.Time.now(), "base_link", "odom")
        br.sendTransform([0.2, 0, 0.2], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_scan", "base_link")
    
    def convertMsg(self,klampt_sensor,frame,stamp = "now"):
        """Not sure why this is needed instead of klampt.io.ros.toMsg()"""
        measurements = klampt_sensor.getMeasurements()
        res = LaserScan()
        if stamp=='now':
            stamp = rospy.Time.now()
        elif isinstance(stamp,(int,float)):
            stamp = rospy.Time(stamp)
        res.header.frame_id = frame
        res.header.stamp = stamp
        res.angle_max = float(klampt_sensor.getSetting("xSweepMagnitude"))
        res.angle_min = -1.0 * res.angle_max
        measurement_count = float(klampt_sensor.getSetting("measurementCount"))
        res.angle_increment = (res.angle_max - res.angle_min)/measurement_count
        res.time_increment = float(klampt_sensor.getSetting("xSweepPeriod"))
        res.range_min = float(klampt_sensor.getSetting("depthMinimum"))
        res.range_max = float(klampt_sensor.getSetting("depthMaximum"))
        res.ranges = measurements
        res.intensities = []
        return res





if __name__ == '__main__':
    print('\n\n\n\n\n running as Main\n\n\n\n\n')
    from matplotlib import pyplot as plt
    module = SensorModule(None, mode='Kinematic')
    api = module.api("main_thread",None,None)
    print("Available cameras:",api.camerasAvailable())
    res = api.getNextPointClouds()
    print("Result of getNextPointClouds",res)
    print("Waiting on Promise...")
    print("Result of await",res.await())
    print("Beginning loop...")
    while module.status != 'terminate':
        time.sleep(1)

    module.get_rgbd_images('realsense_left')
    module.get_point_clouds('realsense_left')
    time.sleep(1)
    cam_image = module.get_rgbd_images('realsense_left')['realsense_left']
    print(cam_image)
    plt.imshow(cam_image[1])
    plt.show()
    cam_o3d = module.get_point_clouds('realsense_left')['realsense_left']
    print(cam_o3d.colors)
    print(np.asarray(cam_o3d.points))
    time.sleep(1)
    module.terminate()
    # b = []
    # for i in range(50):
    #     b = module.get_point_clouds()
    #     print(b.keys())
    #     print(b[b.keys()[0]])
    #     time.sleep(0.08)
    #     if(int(time.time()) % 2 == 0):
    #         plt.close('all')
