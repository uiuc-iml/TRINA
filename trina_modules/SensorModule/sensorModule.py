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
from klampt.math import se3
from klampt import vis
from klampt.model import sensing
from threading import Thread
import threading
import copy
from OpenGL.GLUT import *
from OpenGL.GL import *
from klampt.math import vectorops,so3
from klampt import WorldModel, vis
from klampt.model import sensing
import klampt.io.numpy_convert
import sys,os
try:
    from Settings import trina_settings
    from Jarvis import JarvisAPIModule,JarvisAPI
    from Utils.promise import Promise
except ImportError:
    sys.path.append(os.path.expanduser("~/TRINA"))
    from Settings import trina_settings
    from Jarvis import JarvisAPIModule,JarvisAPI
    from Utils.promise import Promise
from Jarvis import Jarvis

try:
    import tf
    import rospy
    import sensor_msgs
    from sensor_msgs.msg import LaserScan
    _have_ros = True
except ImportError:
    _have_ros = False

import time 
from multiprocessing import Process, Pipe
import os

class CameraData:
    def __init__(self,settings=None):
        if settings is not None:
            self.serial_number = settings["serial_number"]
            self.link = settings["link"]
            calib_fn = settings["transform"]
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
        self.world_transform = None

class SensorModule(JarvisAPIModule):
    """
    This is the primary class of this helper module. It instantiates a motion client and handles the streamed inputs from the realsense camera
    """

    def __init__(self, robot_or_jarvis, cameras= None, mode='Kinematic', ros_active = True):
        import os


        trina_dir = os.path.expanduser("~TRINA")
        if isinstance(robot_or_jarvis,Jarvis):
            self.jarvis = robot_or_jarvis
            self.robot = self.jarvis.robot
        else:
            self.jarvis = None
            self.robot = robot_or_jarvis

        # we first check if the parameters are valid:
        # checking if cameras make sense:
        self.cameras = cameras if cameras is not None else []
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
            self.world = trina_settings.robot_model_load()
            self.temprobot = self.world.robot(0)
            camera_settings = trina_settings.camera_settings()
            for camera in self.cameras:
                if camera not in camera_settings:
                    raise ValueError('invalid camera %s selected. Valid cameras are %s (see TRINA/Settings/).\nPlease update camera selection and try again'%(camera,', '.join(camera_settings.keys())))
                # if the camera is valid, import and configure the camera
                self.active_cameras[camera] = CameraData(camera_settings[camera])
                self.startSimpleThread(self.update_camera,args=(camera,),dt=camera_settings[camera].get('framerate',1.0/30.0),initfunc=self.init_camera,name=camera,dolock=False)
        elif self.mode == 'Kinematic':
            self.world = trina_settings.robot_model_load()
            self.temprobot = self.world.robot(0)
            self.simworld = trina_settings.simulation_world_load()
            self.simrobot = self.simworld.robot(0)
            self.sim = klampt.Simulator(self.simworld)

            camera_settings = trina_settings.camera_settings()
            for camera in camera_settings:
                cam = self.sim.controller(0).sensor(camera)
                if len(cam.name()) > 0: # valid camera
                    print("Updating simulated camera",cam.name(),"to match calibration in TRINA/Settings/")
                    calib_fn = camera_settings["transform"]
                    calib_fn = os.path.join(trina_dir,calib_fn )
                    transform = np.load(open(calib_fn, 'rb'))
                    sensing.set_camera_xform(cam,se3.from_homogeneous(transform))
            
            for camera in self.cameras:
                simcam = self.sim.controller(0).sensor(camera)
                if len(simcam.name()) == 0: #invalid camera
                    all_cams = []
                    i = 0
                    while True:
                        sensori = self.sim.controller(0).sensor(i)
                        if sensori.type() == 'CameraSensor':
                            all_cams.append(sensori.name())
                        if sensori.type() == '':  #out of sensors
                            break
                        i += 1
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
            
            self.startSimpleThread(self.update_sim,0.1,init=self.init_GL,name="update_sim",dolock=False)
        self.startSimpleThread(self.update_robot,dt=1.0/30.0,name="update_robot",dolock=False)

    def api(self,*args,**kwargs):
        return JarvisSensorAPI(self,"sensors",*args,**kwargs)

    def terminate(self):
        JarvisAPIModule.terminate(self)
        #by now all threads should have terminated, so no risk of problems
        for c in self.active_cameras:
            if hasattr(self.active_cameras[c].device,'safely_close'):
                self.active_cameras[c].device.safely_close()
        self.active_cameras = {}
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
                    camera.world_transform = self.camera_transform(camera_name,type='numpy')
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
                        if camera.point_cloud is not None:
                            assert camera.world_transform is not None
                            results[camera_name] = camera.point_cloud.transform(camera.world_transform)
                    ntrigger = len(self.active_cameras) if cameras is None else len(cameras)
                    if len(results) == ntrigger:
                        #fire the callback if all camera data is available
                        promise.callback(results)
                        continue
                newrequests.append(req)
            self.requests = newrequests

    def camera_transform(self, camera_name, type='klampt'):
        #assume update_lock is called?
        camera = self.active_cameras[camera_name]
        link = self.temprobot.link(camera.link)
        T = se3.mul(link.getTransform(),camera.transform)
        if type == 'klampt':
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
            if camera not in self.valid_cameras:
                continue
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
            if camera not in self.valid_cameras:
                #raise an error here?
                continue

            local_pc = self.active_cameras[camera].point_cloud
            if local_pc is not None:
                with self.update_lock:
                    assert self.active_cameras[camera].world_transform is not None
                    world_pc = local_pc.transform(self.active_cameras[camera].world_transform))
            else:
                world_pc = None
            output[camera] = world_pc
        return output

    def update_robot(self):
        try:
            # print(self.jarvis.sensedRobotq(),self.jarvis.sensedRightEETransform(),self.jarvis.sensedLeftEETransform())
            q = self.jarvis.sensedRobotq()
        except Exception as e:
            print('error updating robot state')
            print(e)
            return
        with self.update_lock:
            self.temprobot.setConfig(q)
            
    def safely_close_all(self):
        for camera in self.active_cameras.keys():
            self.active_cameras[camera].safely_close()

    def init_GL(self):
        # GLEW WORKAROUND
        glutInit([])
        glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE |
                            GLUT_DEPTH | GLUT_MULTISAMPLE)
        glutInitWindowSize(1, 1)
        windowID = glutCreateWindow("test")

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
        #assume temprobot is updated
        for k,cam in self.active_cameras.items():
            if cam.want_images or cam.want_point_cloud:
                cam.device.kinematicSimulate(self.simworld, self.dt)
                images = sensing.camera_to_images(cam, image_format='numpy', color_format='channels')
                if cam.want_point_cloud:
                    rgb,depth = imdata
                    local_pc = klampt.model.sensing.image_to_points(depth,rgb,xfov,yfov)
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(local_pc[:, :3])
                    pcd.colors = o3d.utility.Vector3dVector(local_pc[:, 3:])
                    with self.update_lock:
                        cam.images = imaces
                        cam.point_cloud = pcd
                        cam.transform = self.camera_transform(k)
                elif cam.want_images:
                    cam.images = images

        if self.ros_active:
            self.lidar.kinematicSimulate(self.simworld,self.dt)
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
                            assert camera.world_transform is not None
                            results[name] = camera.point_cloud.transform(camera.world_transform)
                #fire the callback if all camera data is available
                if len(requests) == len(cameras):
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



class RealSenseCamera:
    def __init__(self, serial_num):
        import pyrealsense2 as rs
        self.serial_num = serial_num
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.latest_aligned_frame = None
        try:
            self.config.enable_device(serial_num.encode('utf-8'))
            self.config.enable_stream(
                rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(
                rs.stream.color, 640, 480, rs.format.rgb8, 30)
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            # Start streaming
            self.pipeline.start(self.config)
            # we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
            self.pc = rs.pointcloud()
        except Exception as e:
            print(e, 'Invalid Camera Serial Number')
            self.pipeline.stop()
        # atexit.register(self.safely_close)

    def update(self):
        frames = self.pipeline.wait_for_frames()
        if not frames.get_depth_frame() or not frames.get_color_frame():
            return
        # Fetch color and depth frames and align them
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        self.latest_aligned_frame = aligned_frames

    def latest_point_cloud(self):
        """
        Returns the point cloud from the last frame.

        Args:
        Returns:
            transformed_pc : returns the point cloud in the camera's local frame
            as a open3D PointCloud, or None if the data is not available
        """
        if self.latest_aligned_frame is None:
            print("Data Not Available at the moment")
            return None
        # Fetch color and depth frames and align them
        depth_frame = self.latest_aligned_frame.get_depth_frame()
        color_frame = self.latest_aligned_frame.get_color_frame()
        
        # Tell pointcloud object to map to this color frame
        self.pc.map_to(color_frame)
        # Generate the pointcloud and texture mappings
        points = self.pc.calculate(depth_frame)
        vtx = np.asarray(points.get_vertices())
        pure_point_cloud = np.zeros((640*480, 3))
        pure_point_cloud[:, 0] = -vtx['f0']
        pure_point_cloud[:, 1] = -vtx['f1']
        pure_point_cloud[:, 2] = -vtx['f2']
        color_t = np.asarray(color_frame.get_data()).reshape(640*480, 3)/255
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pure_point_cloud)
        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        return point_cloud
    
    def latest_rgbd_images(self):
        if self.latest_aligned_frame is None:
            print("Data Not Available at the moment")
            return None
        depth_frame = self.latest_aligned_frame.get_depth_frame()
        color_frame = self.latest_aligned_frame.get_color_frame()
        return [color_frame.get_data(),depth_frame.get_data()]

    def safely_close(self):
        print('safely closing Realsense camera', self.serial_num)
        self.pipeline.stop()

# from threading import Thread, Lock, RLock
class ZedCamera:
    def __init__(self, serial_num):
        # only import pyzed if running on python3
        if(sys.version_info[0] < 3):
            pass
        else:
            import pyzed.sl as sl
        # Note: This code presumes the zed cameras are fixed w.r.t. the base.
        # Create a Camera object
        self.zed = sl.Camera()
        self.serial_num = serial_num
        self.connected = False
        
        self.point_cloud = sl.Mat()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sdk_verbose = False
        # init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_resolution = sl.RESOLUTION.HD2K
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.set_from_serial_number(serial_num)
        init_params.depth_minimum_distance = 0.20
        init_params.depth_maximum_distance = 40
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.runtime_parameters = sl.RuntimeParameters()
        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print('There was an error while trying to access the zed camera, please review and try again.')
    
    def update(self):
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.connected = True
        else:
            self.connected = False

    def latest_point_cloud(self):
        if not self.connected:
            return None
        self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
        pc = self.point_cloud.get_data()

        float_color = pc[:, :, 3]
        final_shape = float_color.shape
        data = float_color.flatten(order='F').view(np.uint8)
        red = data[::4].reshape(final_shape, order='F')
        green = data[1::4].reshape(final_shape, order='F')
        blue = data[2::4].reshape(final_shape, order='F')
        alpha = data[3::4].reshape(final_shape, order='F')

        color_pic = np.zeros(
            (final_shape[0], final_shape[1], 3), dtype=np.uint8)
        color_pic[:, :, 0] = red
        color_pic[:, :, 1] = green
        color_pic[:, :, 2] = blue
        color_t = np.asarray(color_pic).reshape(-1, 3)/255

        # pc = np.nan_to_num(pc)
        reshaped_pc = np.zeros((pc.shape[0]*pc.shape[1], 3))
        reshaped_pc[:, 0] = pc[:, :, 0].flatten()
        reshaped_pc[:, 1] = pc[:, :, 1].flatten()
        reshaped_pc[:, 2] = pc[:, :, 2].flatten()
        reshaped_pc = np.nan_to_num(reshaped_pc, 0, posinf=0, neginf=0)
        # we must now process color - and here comes a lot of bit shifting

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(reshaped_pc)
        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        # we then finally transform the point cloud to the robot's coordinates:
        return point_cloud
    def latest_rgbd_images(self):
        if not self.connected:
            print("Data Not Available at the moment")
            return None
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT) # Get the left image
        self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH) # Retrieve depth Mat. Depth is aligned on the left image
        return([self.image.get_data(),self.depth.get_data()])
    
    def safely_close(self):
        print('safely closing zed camera ', self.serial_num)
        self.zed.close()



class JarvisSensorAPI(JarvisAPI):
    def __init__(self,sensor_module,*args,**kwargs):
        self.sensor_module = sensor_module
        self.lock = sensor_module.update_lock
        JarvisAPI.__init__(self,*args,**kwargs)

    def camerasAvailable(self):
        with self.lock:
            return list(self.sensor_module.active_cameras.keys())

    def getRgbdImages(self,cameras=None):
        """Returns the RGBD cameras from all cameras corresponding to the
        latest images taken.

        Return:
        --------------
        dict containing (rgb,depth) image pairs.  Each image is a numpy object.
        """
        with self.lock:
            return self.sensor_module.get_rgbd_images(cameras)

    def getNextRgbdImages(self,cameras=None):
        """Returns a Promise for the next RGBD images"""
        with self.lock:
            p = Promise("RGBD image request from "+self._caller_name)
            if isinstance(cameras,str):
                cameras = [cameras]
            self.sensor_module.requests.append((p,'get_rgbd_images',cameras,{}))
            return p

    def getPointClouds(self,cameras=None):
        """Returns the point clouds corresponding to the latest image.
        taken.

        Return:
        --------------
        dict containing point clouds. Each point cloud is expressed in world
        coordinates as Open3D PointCloud objects.
        """
        with self.lock:
            return self.sensor_module.get_point_clouds(cameras)

    def getNextPointClouds(self,cameras=None):
        """Returns a Promise for the next point clouds"""
        with self.lock:
            p = Promise("Point cloud request from "+self._caller_name)
            if isinstance(cameras,str):
                cameras = [cameras]
            self.sensor_module.requests.append((p,'get_point_clouds',cameras,{}))
            return p


if __name__ == '__main__':
    print('\n\n\n\n\n running as Main\n\n\n\n\n')
    from matplotlib import pyplot as plt
    a = Camera_Robot(robot = [],world = [], cameras =['realsense_left'],ros_active = False, use_jarvis = False, mode = 'Physical')
    time.sleep(1)
    zed_overhead = a.get_rgbd_images()['realsense_left']
    plt.imshow(zed_overhead[1])
    plt.show()
    zed_o3d = a.get_point_clouds()['realsense_left']
    print(zed_o3d.colors)
    print(np.asarray(zed_o3d.points))
    time.sleep(1)
    a.safely_close_all()
    # b = []
    # for i in range(50):
    #     b = a.get_point_clouds()
    #     print(b.keys())
    #     print(b[b.keys()[0]])
    #     time.sleep(0.08)
    #     if(int(time.time()) % 2 == 0):
    #         plt.close('all')
