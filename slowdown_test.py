#trying to reproduce the errors we got when performing inverse kinematics:

import time,math
import threading
from Motion.motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import pickle
from pdb import set_trace
from numpy import linalg as LA
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import datetime
import csv 
from threading import Thread
import sys
import json
import traceback
import pandas as pd
import pdb
from tqdm import tqdm
# from Motion.motion import Motion

# we start by loading the dataset`
dataset = pd.read_csv('~/TRINA/Motion/teleoperation_log/log_file_2020_31_04_22_31_42', sep = '|')
robot = MotionClient()
UI_state = {}
init_pos_left = {}
cur_pos_left = {}
init_pos_right = {}
cur_pos_right = {}
startup = True
# res = robot.startup()
# dataset = dataset[dataset.ik_time > 0.2]
targets = []
robot.startup()
for i in tqdm(dataset.index[:100]):
    exec('joints = ' + dataset.loc[i,'target_position'])
    exec('old_joints = ' + dataset.loc[i,'current_position'])
    if(dataset.loc[i,'arm'] == 'left'):
        robot.setLeftLimbPosition(joints)
        time.sleep(0.05)
        targets.append(robot.sensedLeftEETransform())
    else:
        robot.setRightLimbPosition(joints)
        time.sleep(0.05)
        targets.append(robot.sensedRightEETransform())

old_joints_l = 0
old_joints_r = 0
exec('old_joints_l = ' + dataset.loc[0,'current_position'])
exec('old_joints_r = ' + dataset.loc[0,'current_position'])
robot.setLeftLimbPosition(old_joints_l)
robot.setRightLimbPosition(old_joints_r)
for i in dataset.index[:100]:
    exec('joints = ' + dataset.loc[i,'target_position'])
    start = time.time()
    if(dataset.loc[i,'arm'] == 'left'):
        robot.setLeftEEInertialTransform(targets[i],0.025)
    else:
        robot.setRightEEInertialTransform(targets[i],0.025)

    print('execution took',time.time()-start, ', ik took ',dataset.loc[i,'ik_time'],', collision checking took :',dataset.loc[i,'collision_check_time'])
    if(i%2 == 0):
        time.sleep(0.025)
robot.shutdown()



