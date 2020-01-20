# import time,math
# from klampt import vis
# from klampt import WorldModel
# from klampt.model.trajectory import Trajectory
# import threading
# from Motion.motion_client import MotionClient
# from Motion.motion import Motion
# import json
# from multiprocessing import Process, Manager, Pipe
# from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
# import pickle
# from pdb import set_trace
# import time,math
# from numpy import linalg as LA
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# import os
# import datetime
# import csv 
# import time
# import websocket
# from threading import Thread
# import time
# import sys
# import json

from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore

interface = RedisInterface(host="localhost")
interface.initialize()
server = KeyValueStore(interface)

a = server["UI_STATE"]['controllerPositionState']['leftController'].read()
print(a.keys())

