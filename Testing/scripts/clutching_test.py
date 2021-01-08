from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import time
import math

import sys
import os
path = os.path.expanduser('~/TRINA/')
sys.path.append(path)
from Jarvis import Jarvis
sys.path.remove(path)


interface = RedisInterface(host="localhost")
interface.initialize()
server = KeyValueStore(interface)

def setup():
    robot = Jarvis("testing")
    
    leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
    rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)
    robot.setLeftLimbPositionLinear(leftUntuckedConfig,0.1)
    robot.setRightLimbPositionLinear(rightUntuckedConfig,0.1)
    
    try:
        retval = input("Press enter to continue")
    except:
        pass
    
    stime = time.time()
    while time.time() - stime < 15:
        time.sleep(0.1)
        server['UI_STATE']["controllerButtonState"]['leftController']["press"] = [True, False, False, False]
        print("Sending home signal")
    server['UI_STATE']["controllerButtonState"]['leftController']["press"] = [False, False, False, False]

setup()

side = 'left'
    
server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerPosition'] = [0,0,0]
server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerRotation'] = [1,0,0,0,1,0,0,0,1]
time.sleep(1)

dt = 0.02

for i in range(3):
    try:
        retval = input("Press enter to go up")
    except:
        pass
    t = 0
    server['UI_STATE']["controllerButtonState"]['leftController']["squeeze"] = [0, 1, 0, 0]
    for i in range(50):
        server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerPosition'] = [0.1 * t*dt, 0, 0]
        t += 1
        time.sleep(dt)
    try:
        retval = input("Press enter to go down")
    except:
        pass
    
    server['UI_STATE']["controllerButtonState"]['leftController']["squeeze"] = [0, 0, 0, 0]
    for i in range(50):
        server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerPosition'] = [0.1 * t*dt, 0, 0]
        t -= 1
        time.sleep(dt)
