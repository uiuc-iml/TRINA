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
from motionStates import * #state structures
from baseController import Path2d
import os
import TRINAConfig
dirname = os.path.dirname(__file__)
#getting absolute model name
model_name = os.path.join(dirname, "data/TRINA_world_reflex.xml")

def setup():
  vis.show()

def callback():
  #...do stuff to world... #this code is executed at approximately 10 Hz due to the sleep call
  time.sleep(0.01)
  if not vis.shown():
    vis.show(False)         #hides the window if not closed by user

def cleanup():
  #can perform optional cleanup code here
  pass


model_config_length = 43
base_indeces = [0,3]
left_limb_indexes = [10,16]
right_limb_indexes = [27,33]
class KinematicController:
    def __init__(self, model_path = model_name):
        self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        self.base_state = BaseState()
        self.left_gripper_state = GripperState()
        self.limb_velocity_limit = 2.0
        self.finger_velocity_limit = 1.0
        self.dt = 0.004
        self.model_path = model_path
        self.world = WorldModel()
        print("KinematicController: loading world")
        res = self.world.readFile(self.model_path)

        if not res:
        	raise RuntimeError("unable to load model")
        self.robot = self.world.robot(0)
        self.new_state = False
        self.shut_down = False
        self.controlLoopLock = Lock()

    def start(self):
        controlThread = threading.Thread(target = self._controlLoop)
        self.left_limb_state.commandedq = self.left_limb_state.sensedq
        self.left_limb_state.commandType = 0
        self.right_limb_state.commandedq = self.right_limb_state.sensedq
        self.right_limb_state.commandType = 0
        #self.left_gripper_state.
        controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()

    def _controlLoop(self):
        prev_angle = None

        while not self.shut_down:
            loopStartTime = time.time()
            self.controlLoopLock.acquire()
            q_to_be_set = [0.0]

            #base
            if self.base_state.commandType == 1:
                vx = self.base_state.commandedVel[0]
                w = self.base_state.commandedVel[1]
            elif self.base_state.commandType == 0:
                vx = self.base_state.pathFollowingVel
                angle = self.base_state.generatedPath.get_pose(float(self.base_state.pathFollowingIdx)/self.base_state.pathFollowingNumPoints)[2]
                if not prev_angle:
                    w = 0
                else:
                    w = (angle - prev_angle)/self.dt

                prev_angle = angle
                self.base_state.pathFollowingIdx += 1

            dq_global = [vx*math.cos(self.base_state.measuredPos[2])*self.dt,\
                vx*math.sin(self.base_state.measuredPos[2])*self.dt,w*self.dt]

            base_state_q_to_be_set = vectorops.add(self.base_state.measuredPos,dq_global)
            self.base_state.measuredVel = deepcopy(self.base_state.commandedVel)
            self.base_state.measuredPos = deepcopy(base_state_q_to_be_set)

            #left limb
            left_limb_to_be_set = []
            dq_to_be_set = []
            if self.left_limb_state.commandType == 0:
                for i in range(6):
                    command = self.left_limb_state.commandedq[i]
                    old = self.left_limb_state.sensedq[i]           
                    if (command-old) > self.limb_velocity_limit*self.dt:
                        qi = self.limb_velocity_limit*self.dt + old
                    elif (command-old) <  -self.limb_velocity_limit:
                        qi = old-self.limb_velocity_limit*self.dt
                    else:
                        qi = command
                    left_limb_to_be_set.append(qi)
                left_limb_to_be_set,flag = self._limitLimbPosition(left_limb_to_be_set)
                self.left_limb_state.sensedq = deepcopy(left_limb_to_be_set)
                self.left_limb_state.senseddq = vectorops.div(vectorops.sub(left_limb_to_be_set,self.left_limb_state.sensedq),self.dt)
            elif self.left_limb_state.commandType == 1:
                for i in range(6):
                    command = self.left_limb_state.commandeddq[i]
                    if command > self.limb_velocity_limit:
                        dq_to_be_set.append(self.limb_velocity_limit)
                    elif command <  -self.limb_velocity_limit:
                        dq_to_be_set.append(-self.limb_velocity_limit)
                    else:
                        dq_to_be_set.append(command)
                left_limb_to_be_set = vectorops.add(self.left_limb_state.sensedq,vectorops.mul(dq_to_be_set,dt))
                left_limb_to_be_set ,flag = self._limitLimbPosition(left_limb_to_be_set)
                self.left_limb_state.senseddq = vectorops.div(vectorops.sub(dq_to_be_set,self.left_limb_state.sensedq),dt)
                self.left_limb_state.sensedq = deepcopy(left_limb_to_be_set)
            #right limb
            right_limb_to_be_set = []
            dq_to_be_set = []
            if self.right_limb_state.commandType == 0:
                for i in range(6):
                    command = self.right_limb_state.commandedq[i]
                    old = self.right_limb_state.sensedq[i]           
                    if (command-old) > self.limb_velocity_limit*self.dt:
                        qi = self.limb_velocity_limit*self.dt + old
                    elif (command-old) <  -self.limb_velocity_limit:
                        qi = old-self.limb_velocity_limit*self.dt
                    else:
                        qi = command
                    right_limb_to_be_set.append(qi)
                right_limb_to_be_set,flag = self._limitLimbPosition(right_limb_to_be_set)
                self.right_limb_state.sensedq = deepcopy(right_limb_to_be_set)
                self.right_limb_state.senseddq = vectorops.div(vectorops.sub(right_limb_to_be_set,self.right_limb_state.sensedq),self.dt)
            elif right_limb_state.commandType == 1:
                for i in range(6):
                    command = self.right_limb_state.commandeddq[i]
                    if command > self.limb_velocity_limit:
                        dq_to_be_set.append(self.limb_velocity_limit)
                    elif command <  -self.limb_velocity_limit:
                        dq_to_be_set.append(-self.limb_velocity_limit)
                    else:
                        dq_to_be_set.append(command)
                right_limb_to_be_set = vectorops.add(self.right_limb_state.sensedq,vectorops.mul(dq_to_be_set,dt))
                right_limb_to_be_set ,flag = self._limitLimbPosition(right_limb_to_be_set)
                self.right_limb_state.senseddq = vectorops.div(vectorops.sub(dq_to_be_set,self.right_limb_state.sensedq),dt)
                self.right_limb_state.sensedq = deepcopy(right_limb_to_be_set)

            if flag:
                print("KinematicController: limbs at position limits")

            #left gripper
            affineDrive = 1.0
            left_gripper_to_be_set = [0]*17
            left_gripper_to_be_set[4] = self.left_gripper_state.command_finger_set[0]
            left_gripper_to_be_set[5] = self.left_gripper_state.command_finger_set[0]*affineDrive
            left_gripper_to_be_set[9] = self.left_gripper_state.command_finger_set[1]
            left_gripper_to_be_set[10] = self.left_gripper_state.command_finger_set[1]*affineDrive
            left_gripper_to_be_set[13] = self.left_gripper_state.command_finger_set[2]
            left_gripper_to_be_set[14] = self.left_gripper_state.command_finger_set[2]*affineDrive
            left_gripper_to_be_set[4] = self.left_gripper_state.command_finger_set[3]
            left_gripper_to_be_set[8] = -self.left_gripper_state.command_finger_set[3]
            #need to check limits here.... ignoring for now....    
            self.left_gripper_state.sense_finger_set = deepcopy(self.left_gripper_state.command_finger_set)
            #set klampt robot config
            #print(len(self.robot.getConfig()))
            #print(len(base_state_q_to_be_set + [0]*7 +left_limb_to_be_set+[0] + left_gripper_to_be_set + [0] +right_limb_to_be_set+[0]*18))
            self.robot.setConfig(base_state_q_to_be_set + [0]*7 +left_limb_to_be_set+[0] + left_gripper_to_be_set + [0] +right_limb_to_be_set+[0]*18)
            self.new_state = True
            self.controlLoopLock.release()
            elapsedTime = time.time() - loopStartTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass
        #print("KinematicController.controlThread():exited")
    def setLeftLimbConfig(self,q):
        self.left_limb_state.commandedq = deepcopy(q)
        return
    def setRightLimbConfig(self,q):
        self.right_limb_state.commandedq = deepcopy(q)
        return

    def setBaseVelocity(self,q):
        self.base_state.commandType = 1
        self.base_state.commandedVel = deepcopy(q)

    def setBaseTargetPosition(self, q, vel):
        assert len(q) == 3
        self.base_state.commandedTargetPosition = deepcopy(q)
        self.base_state.pathFollowingVel = vel
        self.base_state.generatedPath = Path2d([(0, 0, 0), q])
        self.base_state.pathFollowingIdx = 0
        total_path_time = self.base_state.generatedPath.length/self.base_state.pathFollowingVel
        self.base_state.pathFollowingNumPoints = int(total_path_time/self.dt)
        self.base_state.commandType = 0

    def isBasePathDone(self):
        assert(self.base_state.commandType == 0 or self.base_state.commandType == 2)
        return self.base_state.pathFollowingIdx >= self.base_state.pathFollowingNumPoints

    def setLeftGripperPosition(self,q):
        assert len(q) == 4
        self.left_gripper_state.command_finger_set = deepcopy(q)
        #print(self.left_gripper_state.command_finger_set)
        return
    def getLeftGripperPosition(self):
        return self.left_gripper_state.sense_finger_set
    def getLeftLimbConfig(self):
        return self.left_limb_state.sensedq
    def getLeftLimbVelocity(self):
        return self.left_limb_state.senseddq
    def getRightLimbConfig(self):
        return self.right_limb_state.sensedq
    def getRightLimbVelocity(self):
        return self.right_limb_state.senseddq

    def getBaseVelocity(self):
        return self.base_state.measuredVel

    def getBaseConfig(self):
        return self.base_state.measuredPos
    def newState(self):
        return self.new_state

    def markRead(self):
        self.new_state = False
        return

    def shutdown(self):
        self.shut_down = True
        return

    def _setKlamptModelConfig(self):
    	q = [0] + self.left_limb_state.sensedq + [0, 0] + self.right_limb_state.sensedq + [0]
    	self.robot.setConfig(q)
    	return q

    def getWorld(self):
        return self.world

    def _limitLimbPosition(self,q):
        limited_q = []
        flag = False
        for q,l,u in zip(q,TRINAConfig.limb_position_lower_limits,TRINAConfig.limb_position_upper_limits):
            if q > u:
                q = u
                flag = True
            if q < l:
                q = l
                flag = True
            limited_q.append(q)
        return limited_q, flag

    #need to implemented this....
    def _limitConfigPosition(self,q):
        return q
if __name__=="__main__":
    robot = KinematicController()
    robot.start()
    print('flag')
    startTime = time.time()
    world = robot.getWorld()
    vis.add("world",world)
    vis.show()
    #robot.setBaseTargetPosition([1, 1, math.pi/4], 0.5)
    robot.setBaseTargetPosition([1, 1, math.pi/2], 0.5)
    #robot.setBaseTargetPosition([1, 0, 0], 0.5)
    #robot.setBaseTargetPosition([0, 1, 0], 0.25)
    while not robot.isBasePathDone():
        vis.lock()
        vis.unlock()
        time.sleep(0.02)
    """
    while (time.time()-startTime < 5):
        vis.lock()
        robot.setBaseVelocity([0.1,0])
        #print(robot.get)
        vis.unlock()
        time.sleep(0.02)
        
        print(time.time()-startTime)
    """
    robot.shutdown()
    while True:
        vis.lock()
        vis.unlock()
        time.sleep(0.1)
