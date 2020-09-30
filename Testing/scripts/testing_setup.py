import os
home = os.path.expanduser("~")

trina_dir = os.path.join(home,'TRINA')
os.chdir(trina_dir)

from Jarvis import Jarvis
from SensorModule import Camera_Robot
from Motion import MotionClient
from klampt import WorldModel

def generate_test_jarvis(robot_ip =  'http://localhost:8080',mode = 'Kinematic',components = ['base','left_limb','right_limb','left_gripper'],codename = 'anthrax_lowpoly',world_file = './Motion/data/TRINA_world_anthrax_PointClick.xml'):


    robot = MotionClient(address = robot_ip)
    robot.startServer(mode = mode, components = components,codename = 'anthrax_lowpoly')
    world = WorldModel()
    world.readFile(world_file )
    sensor_module = Camera_Robot(robot = robot,world = world, ros_active = False)
    jarvis = Jarvis(str('devel'),sensor_module)
    return jarvis