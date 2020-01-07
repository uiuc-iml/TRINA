import time,math
from klampt import vis
from klampt import WorldModel
from klampt.model.trajectory import Trajectory
import threading
from Motion.motion_client import MotionClient
from Motion.motion import Motion
import json
from multiprocessing import Process, Manager, Pipe
import pickle
from pdb import set_trace
import time,math
from numpy import linalg as LA
import numpy as np
import os
import datetime
import csv 
import time
from threading import Thread
import time
import sys
import json

robot_ip = '130.126.139.236'
ws_port = 1234

model_name = "Motion/data/TRINA_world_reflex.xml"

roomname = "The Lobby"
zonename = "BasicExamples"
userId=0
roomId=-1
is_closed=0
dt = 1.0/15.0

robot = MotionClient()

while True:
    q = robot.getKlamptSensedPosition()
    time.sleep(1.0/15.0)
