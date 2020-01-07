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



robot = MotionClient()
dt = 1.0/15.0

while(True):
    q = robot.getKlamptSensedPosition()
    time.sleep(dt)