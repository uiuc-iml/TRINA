# This is a quality of life improvement module to enable students to use one of TRINA's arms equipped with an intel realsense cameras for grasping tasks
# UIUC - IML - Feburary 2020
# Author: Joao Marcos Marques - jmc12@illinois.equipped
# This code must be run within the TRINA folder.

import atexit
import time
class Camera_Robot:
    """
    This is the primary class of this helper module. It instantiates a motion client and handles the streamed inputs from the realsense camera
    """
    import numpy as np
    import open3d as o3d
    import pickle
    import klampt
    import numpy
    import time

    # Whenever a new realsense camera is added, please update this dictionary with its serial number
    serial_numbers_dict = {'realsense_right':'620202003661','other_realsense':'620202002883'}



    def __init__(self,robot_ip = 'http://localhost:8080',config_file = {'realsense_right':'camera_config.csv'},cameras = ['realsense_right'], mode = 'Kinematic',components = []):
        """
        Instantiates a camera robot instance

        Args:
            robot_ip (str): the ip_address of the motion server, defauts to localhost for local simulated execution. It expects the format http://localhost:8080
            config_file (dict): A dictionary of camera names to camera configuration file paths containing the transform between the camera and the robot's end effector
            cameras ([str]): A list of strings containing which cameras we are using for this process - Valid entries A.T.M. - 'realsense_right' (future - 'zed')
            mode (str): The mode in which the robot will be executed - "Physical" for controlling the real robot, "Kinematic" for controlling the simulation.
        Returns:
            
    """
        self.valid_cameras = ['realsense_right']
        # we first check if the parameters are valid:
        #checking if cameras make sense:
        self.cameras = cameras
        for camera in cameras:
            if(camera in self.valid_cameras):
                #if the camera is a realsense_right camera, import and configure the camera
                if(camera == 'realsense_right'):
                    import pyrealsense2 as rs
                    self.pipeline = rs.pipeline()
                    self.config = rs.config()
                    self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                    self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
                    self.align_to = rs.stream.color
                    self.align = rs.align(self.align_to)
                    # Start streaming
                    self.pipeline.start(self.config)
                    #we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
                    time.sleep(3)
                    self.pc  = rs.pointcloud()
                    atexit.register(self.safely_close_all)
                    self.realsense_transform = pickle.load(open(config_file[camera],'r'))
            else:
                raise ValueError('invalid camera selected. Please update camera selection and try again')

        # we now verify if the camera configuration file makes sense
        # we then try to connect to the motion_client (we always use the same arm - and we never enable the base for now)
        from Motion.motion_client_python3 import MotionClient        
        self.mode = mode
        self.components = components
        self.robot = MotionClient(address = robot_ip)
        self.robot.startServer(mode = self.mode, components = self.components,codename = 'seed')
        self.left_limb_active = ('left_limb' in self.components)
        self.left_gripper_active = ('left_gripper' in self.components)
        self.right_limb_active = ('right_limb' in self.components)
        self.left_gripper_active = ('right_gripper' in self.components)
        time.sleep(4)
        self.startup = True
        res = self.robot.startup()
        if not res:
            raise ValueError('Failed to connect to the robot!!!!!!')

        def get_point_cloud(self,cameras = ['realsense_right'])
            """
            Returns the point cloud from the referred source in the robot's base frame.

            Args:
                cameras ([str]): A string or list of strings containing which cameras we are using for this process - Valid entries A.T.M. - 'realsense_right' (future - 'zed')
            Returns:
                output {camera:point_cloud} : A dictionary containing an open3D point cloud for each camera string for which a point_cloud is available. Returns 
                None as a string if there is no point cloud available for the requested camera 
            """ 
            output = {}
            if(type(cameras) == str):
                cameras = [cameras]
            elif(type(cameras != list)):
                raise TypeError('Selected cameras must be either a string or a list of strings')
            for camera in cameras:
                if(camera in self.valid_cameras):
                    if(camera == 'realsense_right'):
                        # we then collect this camera's data:
                        try:
                            # Wait for the next set of frames from the camera
                            frames = self.pipeline.wait_for_frames()
                            # Fetch color and depth frames and align them
                            aligned_frames = self.align.process(frames)
                            depth_frame = aligned_frames.get_depth_frame()
                            color_frame = aligned_frames.get_color_frame()
                            if not depth_frame or not color_frame:
                                print("Data Not Available at the moment")
                                output.update{camera:None}
                            # Tell pointcloud object to map to this color frame
                            pc.map_to(color_frame)
                            # Generate the pointcloud and texture mappings
                            points = pc.calculate(depth_frame)
                            vtx = np.asarray(points.get_vertices())
                            pure_point_cloud = np.zeros((640*480,3))
                            pure_point_cloud[:,0] = -vtx['f0']
                            pure_point_cloud[:,1] = -vtx['f1']
                            pure_point_cloud[:,2] = -vtx['f2']
                            color_t = np.asarray(color_frame.get_data()).reshape(640*480,3)/255
                            point_cloud = o3d.geometry.PointCloud()
                            point_cloud.points = o3d.utility.Vector3dVector(pure_point_cloud)
                            point_cloud.colors = o3d.utility.Vector3dVector(color_t)
                            # o3d.visualization.draw_geometries([point_cloud])

            return output











    def safely_close_all(self):
        for camera in self.cameras:
            if(camera == 'realsense_right'):
                self.pipeline.stop()

class RealSenseCamera:

    def __init__(self,serial_num,config_file):
        try:
            self.serial_num = serial_num
            self.config_file = config_file
            self.pipeline = rs.pipeline(serial_num)
            self.config = rs.config()
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            self.align_to = rs.stream.color
            self.align = rs.align(self.align_to)
            # Start streaming
            self.pipeline.start(self.config)
            # we sleep for 3 seconds to stabilize the color image - no idea why, but if we query it soon after starting, color image is distorted.
            self.pc  = rs.pointcloud()
            self.realsense_transform = pickle.load(open(self.config_file,'r'))
        except Exception as e:
            print(e,'Invalid Camera Serial Number')
        atexit.register(self.safely_close)

    def get_point_cloud(self,cameras = ['realsense_right'])
        """
        Returns the point cloud from the referred source in the robot's base frame.

        Args:
            cameras ([str]): A string or list of strings containing which cameras we are using for this process - Valid entries A.T.M. - 'realsense_right' (future - 'zed')
        Returns:
            output {camera:point_cloud} : A dictionary containing an open3D point cloud for each camera string for which a point_cloud is available. Returns 
            None as a string if there is no point cloud available for the requested camera 
        """ 
        output = {}
        if(type(cameras) == str):
            cameras = [cameras]
        elif(type(cameras != list)):
            raise TypeError('Selected cameras must be either a string or a list of strings')
        for camera in cameras:
            if(camera in self.valid_cameras):
                if(camera == 'realsense_right'):
                    # we then collect this camera's data:
                    try:
                        # Wait for the next set of frames from the camera
                        frames = self.pipeline.wait_for_frames()
                        # Fetch color and depth frames and align them
                        aligned_frames = self.align.process(frames)
                        depth_frame = aligned_frames.get_depth_frame()
                        color_frame = aligned_frames.get_color_frame()
                        if not depth_frame or not color_frame:
                            print("Data Not Available at the moment")
                            output.update{camera:None}
                        # Tell pointcloud object to map to this color frame
                        pc.map_to(color_frame)
                        # Generate the pointcloud and texture mappings
                        points = pc.calculate(depth_frame)
                        vtx = np.asarray(points.get_vertices())
                        pure_point_cloud = np.zeros((640*480,3))
                        pure_point_cloud[:,0] = -vtx['f0']
                        pure_point_cloud[:,1] = -vtx['f1']
                        pure_point_cloud[:,2] = -vtx['f2']
                        color_t = np.asarray(color_frame.get_data()).reshape(640*480,3)/255
                        point_cloud = o3d.geometry.PointCloud()
                        point_cloud.points = o3d.utility.Vector3dVector(pure_point_cloud)
                        point_cloud.colors = o3d.utility.Vector3dVector(color_t)
                        # o3d.visualization.draw_geometries([point_cloud])

        return output

    def safely_close(self):
        print('safely closing camera',self.serial_num)
        self.pipeline.stop()

if __name__=='__main__':
    a = Camera_Robot()
    while(True):
        print('aaaaaaaaaa')
