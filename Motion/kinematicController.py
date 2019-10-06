import time
from klampt import *
from klampt.math import vectorops,so3,se3,so2
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
from copy import deepcopy

model_config_length = 10
left_limb_indexes = [1,7]
right_limb_indexes = [9,15]
class KinematicController:
    def __init__(self, model_path = "/data/TRINA_world.xml"):
    	self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        self.base_state = BaseState()
        self.limb_velocity_limit = []

        self.dt = 0.004
    	self.model_path = model_path
    	self.world = WorldModel()
    	res = self.world.readFile(self.model_path)
    	if not res:
    		raise RuntimeError("unable to load model")
		self.robot = self.world.robot(0)
		self.vis.add("world",self.world)
        self.newState = False
        self.shutDown = False
        self.controlLoopLock = Lock()
    def start(self):
    	controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()

    def _controlLoop(self):
    	self.vis.show()
    	while vis.shown() and not self.shutDown:
    		loopStartTime = time.time()
    		vis.lock()
            self.controlLoopLock.acquire()
            q_to_be_set = [0.0]

            #base
            vx = base_state.commandedVel[0]
            w = base_state.commandedVel[1]
            dq_global = [vx*math.cos(base_state.measurePos[2])*self.dt,\
                vx*math.sin(base_state.measurePos[2])*self.dt,w*dt]

            base_state_q_to_be_set = vectorops.add(base_state.measurePos,dq_global)
            base_state.measuredVel = deepcopy(base_state.commandedVel)

            #left limb
            left_limb_to_be_set = []
            if left_limb_state.commandType == 0:
                for i in range(6):
                    command = left_limb_state.commandedq[i]
                    old = left_limb_state.sensedq[i]           
                    if (command-old) > limb_velocity_limit*self.dt:
                        qi = limb_velocity_limit*self.dt + old
                    elif (command-old) <  -limb_velocity_limit:
                        qi = old-limb_velocity_limit*self.dt
                    else:
                        qi = command
                    left_limb_to_be_set.append(qi)
                left_limb_state.sensedq = deepcopy(left_limb_to_be_set)
                left_limb_state.senseddq = vectorops.sub(left_limb_to_be_set,left_limb_state.sensedq)/dt
            elif left_limb_state.commandType == 1:
                for i in range(6):
                    command = left_limb_state.commandeddq[i]
                    if command > limb_velocity_limit:
                        dq_to_be_set = limb_velocity_limit
                    elif command <  -limb_velocity_limit:
                        dq_to_be_set = -limb_velocity_limit
                    else:
                        dq_to_be_set = command
                left_limb_state.senseddq = deepcopy(dq_to_be_set)
                left_limb_state.sensedq = vectorops.add(left_limb_state.sensedq,vectorops.mul(dq_to_be_set,dt))
            #right limb
            right_limb_to_be_set = []
            if right_limb_state.commandType == 0:
                for i in range(6):
                    command = right_limb_state.commandedq[i]
                    old = right_limb_state.sensedq[i]           
                    if (command-old) > limb_velocity_limit*self.dt:
                        qi = limb_velocity_limit*self.dt + old
                    elif (command-old) <  -limb_velocity_limit:
                        qi = old-limb_velocity_limit*self.dt
                    else:
                        qi = command
                    right_limb_to_be_set.append(qi)
                right_limb_state.sensedq = deepcopy(right_limb_to_be_set)
                right_limb_state.senseddq = vectorops.sub(right_limb_to_be_set,right_limb_state.sensedq)/dt
            elif left_limb_state.commandType == 1:
                for i in range(6):
                    command = right_limb_state.commandeddq[i]
                    if command > limb_velocity_limit:
                        dq_to_be_set = limb_velocity_limit
                    elif command <  -limb_velocity_limit:
                        dq_to_be_set = -limb_velocity_limit
                    else:
                        dq_to_be_set = command
                right_limb_state.senseddq = deepcopy(dq_to_be_set)
                right_limb_state.sensedq = vectorops.add(right_limb_state.sensedq,vectorops.mul(dq_to_be_set,dt))

            #set klampt robot config
    		self.robot.setConfig([0.0]+base_state_to_be_set + [0,0] +left_limb_to_be_set+[0,0]+right_limb_to_be_set)
            self.newState = True
            self.controlLoopLock.release()
    		vis.unlock()
    		elaspedTime = time.time() - loopStartTime
    		if elaspedTime < self.dt:
      			time.sleep(self.dt-elapsedTime)
      		else:
      			pass
    def setLeftLimbConfig(self,q):
        self.left_limb_state.commandedq = deepcopy(q)
        return
    def setRightLimbConfig(self,q):
        self.right_limb_state.commandedq = deepcopy(q)
        return

    def setBaseVelocity(self,q):
        self.commandeddq = deepcopy(q)

    def getLeftLimbConfig(self):
        return self.left_limb_state.sensedq
    def getLeftLimbVelocity(self):
        return self.left_limb_state.senseddq
    def getRightLimbConfig(self):
        return self.right_limb_state.sensedq
    def getRightLimbVelocity(self):
        return self.right_limb_state.senseddq

    def newState(self):
        return self.newState

    def markRead(self):
        self.newState = False
        return

    def shutDown(self):
        self.shutDown = True

    def _setKlamptModelConfig(self):
    	q = [0] + self.left_limb_state.sensedq + [0, 0] + self.right_limb_state.sensedq + [0.0]
    	self.robot.setConfig(q)
    	return q


