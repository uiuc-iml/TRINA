import atexit
import numpy as np
import open3d as o3d
import pickle
import klampt
import numpy
import time
from klampt.math import se3
from klampt import vis, Geometry3D
from klampt.model import sensing
from threading import Thread
import threading
import copy
from OpenGL.GLUT import *
from OpenGL.GL import *
from sensor_msgs.msg import LaserScan
from klampt.math import vectorops,so3
from klampt.model import ik, collide
from klampt import WorldModel, vis
import time 
from multiprocessing import Process, Pipe

class Camera_Robot:

    """
    This is the primary class of this helper module. It instantiates a motion client and handles the streamed inputs from the realsense camera
    """

    # Whenever a new realsense camera is added, please update this dictionary with its serial number
    def __init__(self, robot, cameras=['realsense_right'], mode='Kinematic', components=[], world=[], ros_active = True):
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
        # we first check if the parameters are valid:
        # we now verify if the camera configuration file makes sense
        # we then try to connect to the motion_client (we always use the same arm - and we never enable the base for now)
        self.mode = mode
        self.components = components
        self.robot = robot
        self.active_cameras = {}
        self.ros_active = ros_active


            # glutInit ([])
            # glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
            # glutInitWindowSize (1, 1)
            # windowID = glutCreateWindow ("test")

            # reset arms

        self.world = world
        self.simrobot = world.robot(0)
        self.simrobot.setConfig(self.robot.getConfig())
        self.sim = klampt.Simulator(self.world)
        self.simulated_cameras = {}
        self.left_cam = self.sim.controller(0).sensor("left_hand_camera")
        self.right_cam = self.sim.controller(0).sensor("right_hand_camera")
        self.lidar = self.sim.controller(0).sensor("lidar")
        self.system_start = time.time()
        self.simulated_cameras.update(
            {'realsense_right': self.right_cam, 'realsense_left': self.left_cam})
        # vis.add("world",self.world)

        # vis.show()
        # vis.kill()
        # and we start the thread that will update the simulation live:
        self.right_image = []
        self.left_image = []
        self.left_point_cloud = []#np.zeros(shape=(3, 6))
        self.right_point_cloud = []#np.zeros(shape=(3, 6))
        self.simlock = threading.Lock()
        simUpdaterThread = threading.Thread(target=self.update_sim)
        simUpdaterThread.daemon = True
        simUpdaterThread.start()


    def get_rgbd_images(self, cameras=[]):
        if(cameras == []):
            cameras = list(self.active_cameras.keys())
        output = {}
        if(type(cameras) == str):
            cameras = [cameras]
        elif(type(cameras) != list):
            raise TypeError(
                'Selected cameras must be either a string or a list of strings')

        else:
            return {"realsense_right": self.right_image, "realsense_left": self.left_image}


    def update_sim(self):
        time.sleep(1)
        self.dt = 0.01
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
        self.frame_counter = 0
        self.smoothed_time = time.time()
        while(True):
            start_time = time.time()
            # print('updating_sim')
            try:
                q = self.robot.getConfig()
            except Exception as e:
                print('error updating robot state')
                print(e)
                pass
            self.simrobot.setConfig(q)
            # with self.simlock:
            # self.sim.simulate(self.dt)
            self.left_cam.kinematicSimulate(self.world, self.dt)
            self.right_cam.kinematicSimulate(self.world, self.dt) 
            self.left_image = sensing.camera_to_images(
                self.left_cam, image_format='numpy', color_format='channels')
            self.right_image = sensing.camera_to_images(
                self.right_cam, image_format='numpy', color_format='channels')

            self.sim.updateWorld()
            elapsed_time = time.time() - start_time
            self.frame_counter += 1 
            print('Simulation Frequency:{:.2}'.format(1.0/elapsed_time))
            if(self.frame_counter == 10):
                self.frame_counter = 0
                self.smoothed_elapsed_time = time.time()-self.smoothed_time
                print('\n\n Smoothed Simulation Frequency: {:.2} \n\n\n'.format(10/self.smoothed_elapsed_time))
                self.smoothed_time = time.time()
            if(elapsed_time < self.dt):
                time.sleep(self.dt-elapsed_time)




if __name__ == '__main__':
    print('\n\n\n\n\n running as Main\n\n\n\n\n')
    world = WorldModel()
    world.readFile('./data/TRINA_world_anthrax_PointClick.xml')
    robot = world.robot(0)
    a = Camera_Robot(robot,world = world)
    time.sleep(1)
    b = []
    while(True):
        # b = a.get_rgbd_images()
        # time.sleep(0.08)
        k = 0