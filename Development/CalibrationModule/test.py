import os
os.chdir('../../')
# # home = os.path.expanduser("~")
# # trina_dir = os.path.join(home,'TRINA')
# # os.chdir(trina_dir)
# retval = os.getcwd()
# print "Current working directory %s" % retval

from Motion.motion_client import MotionClient
from Jarvis import Jarvis
from SensorModule import Camera_Robot
from klampt import WorldModel,Geometry3D