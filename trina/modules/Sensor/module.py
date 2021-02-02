from future import *
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

from trina.modules.Sensor.api import *

#need to convert lidar_transform to have x pointing forward, y to the left, z up
#Klamp't convention has z pointing forward, x pointing left, y pointing up
klampt_to_ros_lidar = so3.from_matrix([[0,1,0],
                                       [0,0,1],
                                       [1,0,0]])

CAMERA_DEACTIVATION_TIME = 5.0

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
        self.time_want_images = 0
        self.time_want_point_cloud = 0
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
        res = self.jarvis.require('Motion')
        assert res != None
        self.status = 'active' #states are " idle, active"  --- idle means you won't be responding to API calls, so it's best to put this as active

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
                print('\n\n\n')
                print("SensorModule: ROS support requested, but rospy could not be imported")
                print('\n\n\n')
            self.ros_active = False
        else:
            if rospy.get_name() == '/unnamed':
                rospy.init_node("TRINA_SensorModule")
            self.ros_subscribers = []

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
                self.startSimpleThread(self._update_camera,args=(camera,),dt=camera_settings[camera].get('framerate',1.0/30.0),initfunc=self._init_camera,name=camera,dolock=False)
            self.lidar_angle_range = None
            self.lidar_data = None
            self.base_transform = None
            self.lidar_transform = None
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
            xmax = float(self.lidar.getSetting('xSweepMagnitude'))
            self.lidar_angle_range = (-xmax,xmax)
            self.lidar_data = None
            self.base_transform = None
            self.lidar_transform = sensing.get_sensor_xform(self.lidar)
            if self.ros_active:
                self.rosTfBroadcaster = tf.TransformBroadcaster()
            self.system_start = time.time()
            
            self.glutWindowID = None
            self.startSimpleThread(self._update_sim,trina.settings.app_settings("SensorModule")['sim_update_dt'],initfunc=self._init_GL,name="update_sim",dolock=False)
        self.startSimpleThread(self._update_robot,dt=trina.settings.app_settings("SensorModule")['update_robot_dt'],name="update_robot",dolock=False)

    @classmethod
    def apiClass(cls):
        return SensorAPI

    def api(self,other_module,other_comms):
        return SensorAPI(self,other_module,other_comms)

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
        if self.ros_active:
            for sub in self.ros_subscribers:
                sub.unregister()
            self.ros_subscribers = []

    def _init_camera(self,camera_name):
        camera = self.active_cameras[camera_name]
        serial_number = camera.serial_number
        try:
            if camera_name.startswith('realsense'):
                driver = RealSenseCamera(serial_number)
            elif camera_name.startswith('zed'):
                driver = ZedCamera(serial_number)
            else:
                print('SensorModule: Verify camera names, no camera match found!')
                raise TypeError('No compatible camera found!')
            camera.driver = driver
            atexit.register(camera.driver.safely_close)
            print('SensorModule: sucessfully initialized camera',camera_name)
        except Exception as e:
            print('SensorModule: Camera', camera_name,'is currently unavailable. Verify that it is connected and try again \n\n')
            with self.update_lock:
                del self.active_cameras[camera_name]
            self.stopThread(camera_name)

        if self.ros_active:
            #subscribe to /base_scan updates
            self.subscribers.append(rospy.Subscriber('/base_scan',LaserScan,callback=self._on_laser_scan,queue_size=10))
            self.rosTfListener = tf.TransformListener()
            lidar_transform_ros = klampt.io.ros.listen_tf(self.rosTfListener,None,'base_scan','base_link')
            if lidar_transform_ros is None:
                print("SensorModule: WARNING: tf module didn't seem to broadcast base_scan -> base_link?")
            else:
                #need to make sure this is in Klampt coordinate convention
                self.lidar_transform = (so3.mul(lidar_transform_ros[0],so3.inv(klampt_to_ros_lidar)),lidar_transform_ros[1])

    def _update_camera(self,camera_name):
        camera = self.active_cameras[camera_name]
        #determine whether to deactivate camera reading
        if camera.time_want_images < time.time() - CAMERA_DEACTIVATION_TIME:
            camera.want_images = False
        if camera.time_want_point_cloud < time.time() - CAMERA_DEACTIVATION_TIME:
            camera.want_point_cloud = False

        if camera.want_images or camera.want_point_cloud:
            camera.driver.update()
            if camera.want_images:
                self._on_camera_images(camera_name,camera.driver.latest_rgbd_images())
            if camera.want_point_cloud:
                self._on_camera_point_cloud(camera_name,camera.driver.latest_point_cloud())
        if self.ros_active:
            self.base_transform = klampt.io.ros.listen_tf(self.rosTfListener,None,'base_link','odom')
            if self.base_transform is None:
                print("SensorModule: WARNING: tf module didn't seem to broadcast base_link -> odom?")

    def _on_camera_images(self,camera_name,image):
        camera = self.active_cameras[camera_name]
        camera.image = image
        with self.update_lock:
            #check if i need to update any pending requests
            newrequests = []
            for req in self.requests:
                promise,query,cameras,results = req
                if cameras is None or camera_name in cameras:
                    if query == 'get_rgbd_images':
                        results[camera_name] = camera.images
                ntrigger = len(self.active_cameras) if cameras is None else len(cameras)
                if len(results) == ntrigger:
                    #fire the callback if all camera data is available
                    promise.callback(results)
                    continue
                newrequests.append(req)
            self.requests = newrequests

    def _on_camera_point_cloud(self,camera_name,local_pc):
        camera = self.active_cameras[camera_name]
        camera.point_cloud = local_pc
        with self.update_lock:
            camera.world_transform = self.get_camera_transform(camera_name,format='numpy')
            camera.world_point_cloud =  camera.point_cloud.transform(camera.world_transform)
            #print("SensorModule: Camera",camera_name,"world transform",camera.world_transform)
        
    def _on_laser_scan(self,laser_scan):
        if isinstance(laser_scan,list):
            self.lidar_data = laser_scan
        else:
            #ROS LaserScan topic
            self.lidar_angle_range = (laser_scan.angle_min,laser_scan.angle_max)
            self.lidar_data =laser_scan.ranges
            
        self.base_transform = self.get_base_transform()
        with self.update_lock:
            newrequests = []
            for req in self.requests:
                promise,query,cameras,results = req
                if query == 'get_lidar_scan':
                    if self.lidar_data is not None:
                        promise.callback(self.lidar_data)
                        continue
                if query == 'get_lidar_point_cloud':
                    if self.lidar_data is not None:
                        res = self.get_lidar_point_cloud()
                        promise.callback(res)
                        continue
                newrequests.append(req)
            self.requests = newrequests
        
    def get_base_transform(self):
        link = int(self.lidar.getSetting('link'))
        if self.mode == 'Kinematic':
            return self.simrobot.link(link).getTransform()
        else:
            return self.temprobot.link(link).getTransform()

    def get_camera_transform(self, camera_name, format='klampt'):
        #assume update_lock is called?
        camera = self.active_cameras[camera_name]
        link = self.temprobot.link(camera.link)
        T = se3.mul(link.getTransform(),camera.local_transform)
        if format == 'klampt':
            return T
        else:
            return klampt.io.numpy_convert.to_numpy(T,'RigidTransform')

    def _update_robot(self):
        self.jarvis.log_health()
        try:
            # print(self.jarvis.robot.sensedRobotq(),self.jarvis.robot.sensedRightEETransform(),self.jarvis.robot.sensedLeftEETransform())
            q = self.jarvis.robot.sensedRobotq()
        except Exception as e:
            print('SensorModule: error updating robot state')
            print(e)
            return
        with self.update_lock:
            self.temprobot.setConfig(q)

            #check if i need to update any pending requests
            newrequests = []
            for req in self.requests:
                promise,query,cameras,results = req
                if cameras is None:
                    cameras = self.active_cameras.keys()
                if query == 'get_rgbd_images':
                    for camera in cameras:
                        camera.want_images = True
                        camera.time_want_images = time.time()
                elif query == 'get_point_clouds':
                    for camera in cameras:
                        camera.want_point_cloud = True
                        camera.time_want_point_cloud = time.time()
            
            
    def _init_GL(self):
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
            self.ros_base_scan_subscriber = rospy.Publisher("base_scan", LaserScan)

    def _update_sim(self):
        with self.update_lock:
            self.simrobot.setConfig(self.temprobot.getConfig())
        dt = 1.0/30.0 # this is unused in kinematicSimulate
        warn_threshold = 0.5
        #assume temprobot is updated
        for k,cam in self.active_cameras.items():
            t0 = time.time()
            if cam.want_images and cam.time_want_images < t0 - CAMERA_DEACTIVATION_TIME:
                print("Deactivated image simulation of camera",k)
                cam.want_images = False
            if cam.want_point_cloud and cam.time_want_point_cloud < t0 - CAMERA_DEACTIVATION_TIME:
                print("Deactivated point cloud simulation of camera",k)
                cam.want_point_cloud = False
            if cam.want_images or cam.want_point_cloud:
                print("Simulating camera",k)
                cam.device.kinematicSimulate(self.simworld, dt)
                images = sensing.camera_to_images(cam.device, image_format='numpy', color_format='channels')
                if cam.want_point_cloud:
                    if int(cam.device.getSetting("depth")) == 0:
                        print("SensorModule: Simulated camera",k,"does not have depth information")
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
                        self._on_camera_images(k,images)
                        self._on_camera_point_cloud(k,pcd)
                    t1 = time.time()
                    if t1-t0 > warn_threshold:
                        print("Getting camera point clouds took time",t1-t0)
                elif cam.want_images:
                    t1 = time.time()
                    if t1-t0 > warn_threshold:
                        print("Getting camera images took time",t1-t0)
                    self._on_camera_images(k,images)

        self.lidar.kinematicSimulate(self.simworld,dt)
        t0 = time.time()
        self._on_laser_scan(self.lidar.getMeasurements())
        t1 = time.time()
        if t1-t0 > warn_threshold:
            print("Getting Lidar data took time",t1-t0)

        if self.ros_active: 
            print("BROADCASTING LIDAR TO ROS")
            #broadcast to ros so the mapper can run
            t = rospy.Time.now()
            ros_msg = klampt.io.ros.to_SensorMsg(self.lidar,frame="/base_scan",stamp=t)
            self.ros_base_scan_subscriber.publish(ros_msg)

            lidar_transform = (so3.mul(self.lidar_transform[0],klampt_to_ros_lidar),self.lidar_transform[1])
            klampt.io.ros.broadcast_tf(self.rosTfBroadcaster,self.base_transform,frameprefix="base_link",root="odom",stamp=t)
            klampt.io.ros.broadcast_tf(self.rosTfBroadcaster,self.lidar_transform,frameprefix="base_scan",root="base_link",stamp=t)
            
        
    def get_rgbd_images(self, cameras=None):
        """API-call. """
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
            self.active_cameras[camera].time_want_images = time.time()
            output[camera] = self.active_cameras[camera].images
        return output

    def get_point_clouds(self, cameras=None):
        """
        API-call. Returns the point cloud from the referred source in the robot's
        base frame.

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
            self.active_cameras[camera].time_want_point_cloud = time.time()
            local_pc = self.active_cameras[camera].point_cloud
            if local_pc is not None:
                with self.update_lock:
                    assert self.active_cameras[camera].world_transform is not None
                    world_pc = local_pc.transform(self.active_cameras[camera].world_transform)
            else:
                world_pc = None
            output[camera] = world_pc
        return output

    def get_lidar_scan(self):
        return self.lidar_data

    def get_lidar_point_cloud(self):
        if self.lidar_data is None or self.lidar_transform is None or self.base_transform is None:
            return None
        thetamin,thetamax = self.lidar_angle_range
        distances = np.array(self.lidar_data)
        #convert distance readings to evenly spaced points in the space
        angles = np.linspace(thetamin,thetamax,len(self.lidar_data),endpoint=True)
        x = np.sin(angles)
        y = np.zeros(len(angles))
        z = np.cos(angles)
        pts_cols = np.multiply(np.row_stack((x,y,z)),distances)
        Tworld = se3.mul(self.base_transform,self.lidar_transform)
        R = klampt.io.numpy_convert.to_numpy(Tworld[0],'Matrix3')
        t = np.array(Tworld[1])
        return np.dot(R,pts_cols).T + t

class SensorModuleSlow(SensorModule):
    """A sensor module that copies images / point clouds to the Redis server.
    Slower than SensorModule but interprocess communication works.
    """
    def __init__(self,*args,**kwargs):
        SensorModule.__init__(self,*args,**kwargs)
        self.verbose = 1
        self.server = self.jarvis.server
        self.server.set(['Sensor','cameraNames'],list(self.active_cameras.keys()))
        self.cameradict = {}
        for cam in self.active_cameras:
            self.cameradict[cam] = dict()
            self.cameradict[cam]['requestImagesTime'] = 0
            self.cameradict[cam]['requestPointCloudTime'] = 0
            self.cameradict[cam]['images'] = {'rgb':None,'depth':None}
            self.cameradict[cam]['pointCloud'] = None
        self.server.set(['Sensor','cameras'],self.cameradict)
        self.server.set(['Sensor','lidar'],{'scan':None,'pointCloud':None})
        self.startSimpleThread(self.checkRequests,0.1,name='checkRequests',dolock=False)
        
    def checkRequests(self):
        aliveTime = time.time() - CAMERA_DEACTIVATION_TIME
        for cam in self.active_cameras:
            t = self.server.get(['Sensor','cameras',cam,'requestImagesTime'])
            if t > aliveTime:
                self.active_cameras[cam].want_images = True
                self.active_cameras[cam].time_want_images = t
            else:
                self.active_cameras[cam].want_images = False
            t = self.server.get(['Sensor','cameras',cam,'requestPointCloudTime'])
            if t > aliveTime:
                self.active_cameras[cam].want_point_cloud = True
                self.active_cameras[cam].time_want_point_cloud = t
            else:
                self.active_cameras[cam].want_point_cloud = False

    @classmethod
    def apiClass(cls):
        return SensorAPISlow

    def api(self,other_module,other_comms):
        return SensorAPISlow(other_module,other_comms)

    def _on_camera_images(self,camera,images):
        SensorModule._on_camera_images(self,camera,images)
        if len(images) == 2:  #rgb and depth
            self.server.set(['Sensor','cameras',camera,'images','rgb'],images[0])
            self.server.set(['Sensor','cameras',camera,'images','depth'],images[1])
        else:
            self.server.set(['Sensor','cameras',camera,'images','rgb'],images)

    def _on_camera_point_cloud(self,camera,pc):
        SensorModule._on_camera_point_cloud(self,camera,pc)
        self.server.set(['Sensor','cameras',camera,'pointCloud'],pc)

    def _on_laser_scan(self,scan):
        SensorModule._on_laser_scan(self,scan)
        self.server.set(['Sensor','lidar','scan'],scan)
        self.server.set(['Sensor','lidar','pointCloud'],self.get_lidar_point_cloud())


if __name__ == '__main__':
    module = SensorModuleSlow(None, mode='Kinematic')
    while True:
        time.sleep(1)
    exit(0)

    print('\n\n\n\n\n running as Main\n\n\n\n\n')
    from matplotlib import pyplot as plt
    module = SensorModule(None, mode='Kinematic')
    
    api = module.api("main_thread",None)
    print("Available cameras:",api.camerasAvailable())
    res = api.getNextPointClouds()
    print("Result of getNextPointClouds",res)
    print("Waiting on Promise...")
    print("Result of wait",res.wait())
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
