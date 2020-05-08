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
from motion_client import MotionClient
from klampt import vis, Geometry3D
from klampt.model import sensing
from motion import *
from OpenGL.GLUT import *
from threading import Thread
import copy

class Camera_Robot:

    """
    This is the primary class of this helper module. It instantiates a motion client and handles the streamed inputs from the realsense camera
    """


    # Whenever a new realsense camera is added, please update this dictionary with its serial number



    def __init__(self,robot,cameras = ['realsense_right'], mode = 'Kinematic',components = [],world = []):
        """
        Instantiates a camera robot instance

        Args:
            robot_ip (str): the ip_address of the motion server, defauts to localhost for local simulated execution. It expects the format http://localhost:8080
            config_file (dict): A dictionary of camera names to camera configuration file paths containing the transform between the camera and the robot's end effector
            cameras ([str]): A list of strings containing which cameras we are using for this process - Valid entries A.T.M. - ['realsense_right','realsense_left','zed_torso','zed_back'] 
            mode (str): The mode in which the robot will be executed - "Physical" for controlling the real robot, "Kinematic" for controlling the simulation.
            components ([str]): A list of strings indicating which components of the robot you wish to command - valid entries : ['base','left_limb','right_limb','left_gripper']
        Returns:
            
    """
        self.serial_numbers_dict = {'realsense_right':"620202003661",'realsense_left':'620202002883','zed_torso':24560,'zed_back':24545}
        self.config_files_dict = {'realsense_right':'./Sensors/realsense_right_config.p','realsense_left':'./Sensors/realsense_left_config.p',
            'zed_torso':'./Sensors/zed_torso_config.p','zed_back':'./Sensors/zed_back_config.p'}
        self.valid_cameras = ['realsense_right','realsense_left','zed_torso','zed_back']
        # we first check if the parameters are valid:
        #checking if cameras make sense:
        self.cameras = cameras
        # we now verify if the camera configuration file makes sense
        # we then try to connect to the motion_client (we always use the same arm - and we never enable the base for now)
        self.mode = mode
        self.components = components
        self.robot = robot
        self.active_cameras = {}
        if(self.mode == 'physical'):
            import pyrealsense2 as rs
            import pyzed.sl as sl
            for camera in cameras:
                if(camera in self.valid_cameras):
                    #if the camera is a realsense_right camera, import and configure the camera
                    try:
                        self.active_cameras.update({camera:Camera_Sensors(camera,self.serial_numbers_dict,self.config_files_dict,self.robot)})
                        atexit.register(self.active_cameras[camera].safely_close)
                    except Exception as e:
                        print('This camera ',camera,' is currently unavailable. Verify that it is connected and try again \n\n')
                else:
                    raise ValueError('invalid camera selected. Please update camera selection and try again')
        elif(self.mode == 'Kinematic'):

            # glutInit ([])
            # glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
            # glutInitWindowSize (1, 1)
            # windowID = glutCreateWindow ("test")

            # reset arms

            self.world = world
            self.simrobot = world.robot(0)
            self.simrobot.setConfig(self.robot.getKlamptSensedPosition())
            self.sim = klampt.Simulator(self.world)
            self.simulated_cameras = {}
            self.left_cam = self.sim.controller(0).sensor("left_hand_camera")
            self.right_cam = self.sim.controller(0).sensor("right_hand_camera")
            self.simulated_cameras.update({'realsense_right':self.right_cam,'realsense_left':self.left_cam})
            vis.add("world",self.world)

            vis.show()
            vis.kill()
            # and we start the thread that will update the simulation live:
            simUpdaterThread = threading.Thread(target=self.update_sim)
            simUpdaterThread.start()   
                
    def get_point_clouds(self,cameras = []):
        """
        Returns the point cloud from the referred source in the robot's base frame.

        Args:
            cameras ([str]): A string or list of strings containing which cameras we are using for this process - Valid entries A.T.M. - 'realsense_right' (future - 'zed')
        Returns:
            output {camera:point_cloud} : A dictionary containing an open3D point cloud for each camera string for which a point_cloud is available. Returns 
            None as a string if there is no point cloud available for the requested camera 
        """ 
        if(cameras == []):
            cameras = list(self.active_cameras.keys())
        output = {}
        if(type(cameras) == str):
            cameras = [cameras]
        elif(type(cameras) != list):
            raise TypeError('Selected cameras must be either a string or a list of strings')
        if(self.mode == 'physical'):
            for camera in cameras:
                if(camera in self.valid_cameras):
                    output.update({camera:self.active_cameras[camera].get_point_cloud()})
            return output
        else:

            lpc = sensing.camera_to_points(self.left_cam, points_format='numpy', all_points=False, color_format='channels')
            rpc = sensing.camera_to_points(self.right_cam, points_format='numpy', all_points=False, color_format='channels')

            left_pcd = o3d.geometry.PointCloud()
            left_pcd.points = o3d.utility.Vector3dVector(lpc[:,:3])
            left_pcd.colors = o3d.utility.Vector3dVector(lpc[:,3:])
            right_pcd = o3d.geometry.PointCloud()
            right_pcd.points = o3d.utility.Vector3dVector(rpc[:,:3])
            right_pcd.colors = o3d.utility.Vector3dVector(rpc[:,3:])
            # we finally transform the point clouds:
            try:

                Rrotation = self.Rrotation
                Rtranslation = self.Rtranslation
                Lrotation = self.Lrotation
                Ltranslation = self.Ltranslation
                REE_transform = np.array(se3.homogeneous((Rrotation,Rtranslation)))
                LEE_transform = np.array(se3.homogeneous((Lrotation,Ltranslation)))
                self.realsense_transform = np.eye(4)
                # we then multiply this transform with the transform between the end effector and the camera
                final_Rtransform = np.matmul(REE_transform,self.realsense_transform)
                final_Ltransform = np.matmul(LEE_transform,self.realsense_transform)
                # we then invert this transform using klampt se3
                Rft = se3.from_homogeneous(final_Rtransform)
                Rinverted_transform = np.array(se3.homogeneous(se3.inv(Rft)))
                Lft = se3.from_homogeneous(final_Ltransform)
                Linverted_transform = np.array(se3.homogeneous(se3.inv(Lft)))
                
                # we then apply this transform to the point cloud
                Rtransformed_pc = left_pcd.transform(Rinverted_transform)
                Ltransformed_pc = right_pcd.transform(Linverted_transform)
                return {"realsense_right":Rtransformed_pc,"realsense_left":Ltransformed_pc}
            except Exception as e:
                print(e)
                print('failed to communicate with robot')
                return "Failed to Communicate"
    def get_rgbd_images(self,cameras = []):
        if(cameras == []):
            cameras = list(self.active_cameras.keys())
        output = {}
        if(type(cameras) == str):
            cameras = [cameras]
        elif(type(cameras) != list):
            raise TypeError('Selected cameras must be either a string or a list of strings')
        if(self.mode == 'physical'):
            for camera in cameras:
                if(camera in self.valid_cameras):
                    output.update({camera:self.active_cameras[camera].get_point_cloud()})
            return output
        else:
            right_image = sensing.camera_to_images(self.right_cam, image_format= 'numpy', color_format='channels')
            left_image = sensing.camera_to_images(self.left_cam, image_format= 'numpy', color_format='channels')


            return {"realsense_right":right_image,"realsense_left":left_image}


    def safely_close_all(self):
        for camera in self.active_cameras.keys():
            self.active_cameras[camera].safely_close()

    def update_sim(self):
        dt = 0.1
        glutInit ([])
        glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
        glutInitWindowSize (1, 1)
        windowID = glutCreateWindow ("test")
        while(True):
            q = self.robot.getKlamptSensedPosition()
            self.Rrotation,self.Rtranslation = self.robot.sensedRightEETransform()
            self.Lrotation,self.Ltranslation = self.robot.sensedLeftEETransform()

            self.simrobot.setConfig(q)
            self.left_cam.kinematicSimulate(self.world, dt)
            self.right_cam.kinematicSimulate(self.world, dt)
            self.sim.updateWorld()
            time.sleep(dt)

class RealSenseCamera:

    def __init__(self,serial_num,config_file,robot,end_effector = 'right'):
        try:
            self.serial_num = serial_num
            self.config_file = config_file
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_device(serial_num.encode('utf-8'))
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            # Start streaming
            self.pipeline.start(self.config)
            # we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
            self.pc  = rs.pointcloud()
            self.realsense_transform = pickle.load(open(self.config_file,'rb'))
            self.robot = robot
            self.end_effector = end_effector
        except Exception as e:
            print(e,'Invalid Camera Serial Number')
            self.pipeline.stop()
        # atexit.register(self.safely_close)

    def get_point_cloud(self):
        """
        Returns the point cloud from the referred source in the robot's base frame.

        Args:
        Returns:
            transformed_pc : returns the point cloud for this camera in the robot base's coordinate frame. or None if the data is not available
        """ 
        # try:
            # Wait for the next set of frames from the camera
        frames = self.pipeline.wait_for_frames()
        # Fetch color and depth frames and align them
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            print("Data Not Available at the moment")
            return None
        # Tell pointcloud object to map to this color frame
        self.pc.map_to(color_frame)
        # Generate the pointcloud and texture mappings
        points = self.pc.calculate(depth_frame)
        vtx = np.asarray(points.get_vertices())
        pure_point_cloud = np.zeros((640*480,3))
        pure_point_cloud[:,0] = -vtx['f0']
        pure_point_cloud[:,1] = -vtx['f1']
        pure_point_cloud[:,2] = -vtx['f2']
        color_t = np.asarray(color_frame.get_data()).reshape(640*480,3)/255
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(pure_point_cloud)
        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        # we then obtain the transform for the arm where this is attached:
        if(self.end_effector == 'right'):
            rotation,translation = self.robot.sensedRightEETransform()
        elif(self.end_effector == 'left'):
            rotation,translation = self.robot.sensedLeftEETransform()
        EE_transform = np.array(se3.homogeneous((rotation,translation)))
        # we then multiply this transform with the transform between the end effector and the camera
        final_transform = np.matmul(EE_transform,self.realsense_transform)
        # we then invert this transform using klampt se3
        ft = se3.from_homogeneous(final_transform)
        inverted_transform = np.array(se3.homogeneous(se3.inv(ft)))
        
        # we then apply this transform to the point cloud
        transformed_pc = point_cloud.transform(inverted_transform)
            

            # o3d.visualization.draw_geometries([point_cloud])
        # except Exception as e:
        #     print('Something went wrong with our camera system! Please Try Again!')
        #     return None
        return transformed_pc

    def safely_close(self):
        print('safely closing Realsense camera',self.serial_num)
        self.pipeline.stop()

class ZedCamera:
    def __init__(self,serial_num,config_file):
        ### Note: This code presumes the zed cameras are fixed w.r.t. the base.
        # Create a Camera object
        self.zed = sl.Camera()
        self.serial_num = serial_num
        self.transform = pickle.load(open(config_file,'rb'))
        # we then create the inverse transform
        klampt_transforms = se3.from_homogeneous(self.transform)
        self.inverted_transform = np.array(se3.homogeneous(se3.inv(klampt_transforms)))
        
        self.point_cloud = sl.Mat()
        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.sdk_verbose = False
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.set_from_serial_number(serial_num)
        init_params.depth_minimum_distance = 0.20
        init_params.depth_maximum_distance = 40
        self.runtime_parameters = sl.RuntimeParameters()
        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print('There was an error while trying to access the zed camera, please revie and try again.')

    def get_point_cloud(self):
        self.zed.grab(self.runtime_parameters)
        self.zed.retrieve_measure(self.point_cloud,sl.MEASURE.XYZRGBA)
        pc = self.point_cloud.get_data()
        
        float_color = pc[:,:,3]
        final_shape = float_color.shape
        data = float_color.flatten(order = 'F').view(np.uint8)
        red = data[::4].reshape(final_shape,order = 'F')
        green = data[1::4].reshape(final_shape,order = 'F')
        blue = data[2::4].reshape(final_shape,order = 'F')
        alpha = data[3::4].reshape(final_shape,order = 'F')

        color_pic = np.zeros((final_shape[0],final_shape[1],3),dtype = np.uint8)
        color_pic[:,:,0] = red
        color_pic[:,:,1] = green
        color_pic[:,:,2] = blue
        color_t = np.asarray(color_pic).reshape(-1,3)/255

        # pc = np.nan_to_num(pc)
        reshaped_pc = np.zeros((pc.shape[0]*pc.shape[1],3))
        reshaped_pc[:,0] = pc[:,:,0].flatten()
        reshaped_pc[:,1] = pc[:,:,1].flatten()
        reshaped_pc[:,2] = pc[:,:,2].flatten()
        reshaped_pc = np.nan_to_num(reshaped_pc,0,posinf = 0, neginf = 0)
        # we must now process color - and here comes a lot of bit shifting

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(reshaped_pc)
        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
        # we then finally transform the point cloud to the robot's coordinates: 
        transformed_pc = point_cloud.transform(self.inverted_transform)
        return transformed_pc

    def safely_close(self):
        print('safely closing zed camera ',self.serial_num)
        self.zed.close()
    
class Camera_Sensors:
    def __init__(self,camera_name,serial_numbers_dict,config_files_dict,robot,end_effector = 'right'):
        try:
            if(camera_name.startswith('realsense')):
                if(camera_name.endswith('right')):
                    self.camera = RealSenseCamera(serial_numbers_dict[camera_name],config_files_dict[camera_name],robot,end_effector='right')
                elif(camera_name.endswith('left')):
                    self.camera = RealSenseCamera(serial_numbers_dict[camera_name],config_files_dict[camera_name],robot,end_effector='left')

            elif(camera_name.startswith('zed')):
                self.camera = ZedCamera(serial_numbers_dict[camera_name],config_files_dict[camera_name])
            else:
                print('Verify camera names, no camera match found!')
                raise TypeError('No compatible camera found!')
        except Exception as e:
            print('there was an error ',e,'while trying to initialize camera',camera_name)
    def get_point_cloud(self):
        return self.camera.get_point_cloud()
    def safely_close(self):
        self.camera.safely_close()

if __name__=='__main__':
    from matplotlib import pyplot as plt
    a = Camera_Robot()
    time.sleep(1)
    b = []
    for i in range(50):
        b = a.get_point_clouds()
        print(b.keys())
        print(b[b.keys()[0]])
        time.sleep(0.08)
        if(int(time.time())%2 == 0):
            plt.close('all')
    