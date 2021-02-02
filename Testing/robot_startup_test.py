import time,math
# from klampt import vis
# from klampt import WorldModel
# from klampt.model.trajectory import Trajectory
import threading
from Motion.motion_client import MotionClient
robot_ip =  'http://172.16.158.123:8080'

robot = MotionClient(address = robot_ip)

time.sleep(1)

robot.startServer(mode = 'Kinematic', components = ['left_limb'])
robot.startup()

time.sleep(5)
print(robot.sensedLeftLimbPosition())

robot.shutdown()