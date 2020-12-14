import time
import sys
import numpy as np
from copy import copy
sys.path.append('../../Motion/')
from motion_client import MotionClient
server_address = 'http://10.0.242.158:8080' 
server_address = server_address
robot =  MotionClient(address = server_address)


robot.startServer(mode = 'Physical',components = ['left_limb'],codename = 'bubonic')
robot.startup()
time.sleep(1)
K = np.array([[0.0,0.0,0.0,0.0,0.0,0.0],\
    [0.0,0.0,0.0,0.0,0.0,0.0],\
    [0.0,0.0,0.0,0.0,0.0,0.0],\
    [0.0,0.0,0.0,20000.0,0.0,0.0],\
    [0.0,0.0,0.0,0.0,20000.0,0.0],\
    [0.0,0.0,0.0,0.0,0.0,20000.0]])

m = np.eye(6)*2.0
B = np.eye(6)*200.0
initialT = copy(robot.sensedLeftEETransform())
robot.setLeftEETransformImpedance(initialT,K,m,B,deadband = [1.0,1.0,1.0,0.5,0.5,0.5])

while (True):
    time.sleep(0.1)
    
robot.shutdown()