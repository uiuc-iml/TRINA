import time
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt import vis
from klampt.model import trajectory
from klampt.model import coordinates
from klampt.model import ik
from klampt.model import collide
import math
import numpy as np
from threading import Thread, Lock
import threading

class KinematicController:
    def __init__(self, model_path = "/data/TRINA_world.xml"):
    	self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        self.limb_velocity_limits = []

        self.dt = 0.004
    	self.model_path = model_path
    	self.world = WorldModel()
    	res = self.world.readFile(self.model_path)
    	if not res:
    		raise RuntimeError("unable to load model")
		self.robot = self.world.robot(0)
		self.vis.add("world",self.world)

    def start(self):
    	controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()

    def _controlLoop(self):
    	self.vis.show()
    	while vis.shown():
    		loopStartTime = time.time()
    		vis.lock()

    		#left limb:





    		vis.unlock()
    		elaspedTime = time.time() - loopStartTime
    		if elaspedTime < self.dt:
      			time.sleep(self.dt-elapsedTime)
      		else:
      			pass

    def _setKlamptModelConfig(self):
    	q = [0] + self.left_limb_state.sensedq + [0, 0] + self.right_limb_state.sensedq + [0.0]
    	self.robot.setConfig(q)
    	return q


