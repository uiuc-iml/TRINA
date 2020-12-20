from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import pprint
import json
import os
import signal
import sys
import time,math
from threading import Thread, Lock
import threading
from limbController import LimbController
from baseController import BaseController
from kinematicController import KinematicController
import TRINAConfig #network configs and other configs
from motionStates import * #state structures
#import convenienceFunctions 
from copy import deepcopy
from klampt.math import vectorops,so3
from klampt import vis
##Two different modes:
#"Physical" == actual robot
#"Kinematic" == Klampt model
armFlag = True

class SimpleEcho(WebSocket):

    def handleMessage(self):
        # echo message back to client
        obj = json.loads(self.data)
        print(obj)
        # print(obj["controllerPositionState"]["leftController"]["controllerPosition"][1])
        # print(obj["controllerPositionState"]["rightController"]["controllerPosition"][1])
        if obj["controllerPositionState"]["leftController"]["controllerPosition"][1]>0 and obj["controllerPositionState"]["rightController"]["controllerPosition"][1]<0:
            robot = Motion(mode = 'Kinematic')
            res = robot.startup()
            if res:
                startTime = time.time()
            else:
                raise RuntimeError('not started')

            ###Physical mode
            leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
            rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)
            leftGoalConfig = [-1.5596893469439905, -2.3927437267699183, -0.547968864440918, 3.699364348048828, -0.945113484059469, 0.09734535217285156]
            rightGoalConfig = [0.3436760902404785, 0.41109005987133784, 1.171091381703512, -0.38271458566699224, 1.606001377105713, -0.09720212617983037]
            startTime = time.time()
            world = robot.getWorld()
            vis.add("world",world)
            vis.show()
            robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
            robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
            while (time.time()-startTime < 5):
                vis.lock()
                robot.setBaseVelocity([0.0,0.0])
                #print(robot.get)
                time.sleep(0.01)
                vis.unlock()
                print(time.time()-startTime)

            robot.setLeftLimbPositionLinear(leftGoalConfig,10)
            robot.setRightLimbPositionLinear(rightGoalConfig,10)
            while (time.time()-startTime < 15):
                vis.lock()
                robot.setBaseVelocity([0.05,-0.1])
                #print(robot.get)
                time.sleep(0.01)
                vis.unlock()
                print(time.time()-startTime)

            robot.setBaseVelocity([0.0,0.0])    
            time.sleep(10)
            robot.shutdown()

    def handleConnected(self):
        print(self.address, 'connected')
        


    def handleClose(self):
        print(self.address, 'closed')


class Motion:

    def __init__(self, mode = 'Kinematic',model_path = "data/TRINA_world.xml"):
        self.mode = mode
        if self.mode == "Kinematic":
            self.simulated_robot = KinematicController(model_path)
        elif self.mode == "Physical":
            ##The individual components
            if armFlag:
                self.left_limb = LimbController(TRINAConfig.left_limb_address,gripper=False,gravity = TRINAConfig.left_limb_gravity_upright)
                self.right_limb = LimbController(TRINAConfig.right_limb_address,gripper=False,gravity = TRINAConfig.right_limb_gravity_upright)
            self.base = BaseController()
            
            self.currentGravityVector = [0,0,-9.81]  ##expressed in the robot base local frame, with x pointint forward and z up
            ##TODO: Add other components
        else:
            raise RuntimeError('Wrong mode specified') 
        self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        self.base_state = BaseState()
        self.startTime = time.time()
        self.t = 0 #time since startup
        self.startUp = False
        self.dt = 0.002
        self.automaticMode = False #if in this mode, a planner or recorded motion can take over. Can be interruped by stopMotion()
        self.stopMotionFlag = False
        self.stopMotionSent = False
        self.shutdownFlag = False
        self.controlLoopLock = Lock()
        signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl-c)

    def sigint_handler(self, signum, frame):
        assert(signum == signal.SIGINT)
        print("SIGINT caught...shutting down the api!")
        self.shutdown()

    def time(self):
        """Returns the time since robot startup, in s"""
        return self.t

    def startup(self):
        """Starts up all the individual components and the main control thread"""
        """After starting, all components stay where they are and update their positions immediately"""

        if self.mode == "Kinematic":
            self.simulated_robot.start()
        elif self.mode == "Physical":
        #Set the correct gravity vector on startup....
            if armFlag:
                tilt_angle = 0.0
                R_tilt = so3.from_axis_angle(([0,1,0],tilt_angle))
                R_local_global_left = so3.mul(R_tilt,TRINAConfig.R_local_global_upright_left)
                R_local_global_right = so3.mul(R_tilt,TRINAConfig.R_local_global_upright_right)
                #gravity_left = so3.apply(so3.inv(R_local_global_left),[0,0,-9.81])
                #gravity_right = so3.apply(so3.inv(R_local_global_right),[0,0,-9.81])
                #self.left_limb.setGravity(gravity_left)
                #self.right_limb.setGravity(gravity_right)

                res = self.left_limb.start()
                time.sleep(1)
                if res == False:
                    #better to replace this with logger
                    print("motion.startup(): ERROR, left limb start failure.")
                    return False
                else:
                    print("motion.startup(): left limb started.")
                    self.left_limb_state.sensedq = self.left_limb.getConfig()[0:6]
                    self.left_limb_state.senseddq = self.left_limb.getVelocity()[0:6]
                    self.left_limb_state.sensedWrench =self.left_limb.getWrench()

                res = self.right_limb.start()
                time.sleep(1)    
                if res == False:
                    #better to replace this with logger
                    print("motion.startup(): ERROR, left limb start failure.")
                    return False
                else:
                    print("motion.startup(): left limb started.")
                    self.right_limb_state.sensedq = self.right_limb.getConfig()[0:6]
                    self.right_limb_state.senseddq = self.right_limb.getVelocity()[0:6]
                    self.right_limb_state.sensedWrench = self.right_limb.getWrench()
            # start the other components        
            self.base.start()
            # TODO: add more components...

        controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()
        self.startUp = True
        return self.startUp

    def _controlLoop(self):
        print("motion.controlLoop(): controlLoop started.")
        while not self.shutdownFlag:
            loopStartTime = time.time()
            self.t = time.time() - self.startTime
            ###lock the thread 
            self.controlLoopLock.acquire()
            if self.mode == "Physical":
                if self.stopMotionFlag:
                    if not self.stopMotionSent: #send only once to avoid drifting...
                        if armFlag:
                            self.left_limb.stopMotion()
                            self.right_limb.stopMotion()
                        self.base.stopMotion()
                        self.stopMotionSent = True #unused
                else:
                    ###update current state
                    if self.base.newState():
                        self.base_state.measuredVel = self.base.getMeasuredVelocity()
                        self.base_state.measuredPos = self.base.getPosition()
                        self.base.markRead()
                    if armFlag:
                        if self.left_limb.newState():
                            self.left_limb_state.sensedq = self.left_limb.getConfig()[0:6]
                            self.left_limb_state.senseddq = self.left_limb.getVelocity()[0:6]
                            self.left_limb_state.sensedWrench =self.left_limb.getWrench()
                            self.left_limb.markRead()
                        if self.right_limb.newState():
                            self.right_limb_state.sensedq = self.right_limb.getConfig()[0:6]
                            self.right_limb_state.senseddq = self.right_limb.getVelocity()[0:6]
                            self.right_limb_state.sensedWrench = self.right_limb.getWrench()
                            self.right_limb.markRead()

                    ###send commands
                    if armFlag:

                        if self.left_limb_state.commandQueue:
                            if self.left_limb_state.commandType == 0:
                                if len(self.left_limb_state.commandedqQueue) > 0:
                                    if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.ur5e_control_rate):
                                        self.left_limb.setConfig(self.left_limb_state.commandedqQueue.pop(0) + [0.0])
                                        self.left_limb_state.lastCommandQueueTime = time.time()
                            elif self.left_limb_state.commandType == 1:
                                if len(self.left_limb_state.commandeddqQueue) > 0:
                                    if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.ur5e_control_rate):
                                        self.left_limb.setVelocity(self.left_limb_state.commandeddqQueue.pop(0) + [0.0])
                                        self.left_limb_state.lastCommandQueueTime = time.time()
                        else:
                            if not self.left_limb_state.commandSent:
                                ###setting position will clear velocity commands
                                if self.left_limb_state.commandType == 0:
                                    self.left_limb.setConfig(self.left_limb_state.commandedq+[0.0])
                                elif self.left_limb_state.commandType == 1:
                                    self.left_limb.setVelocity(self.left_limb_state.commandeddq + [0.0])
                                self.left_limb_state.commandSent = True

                        if self.right_limb_state.commandQueue:
                            if self.right_limb_state.commandType == 0:
                                if len(self.right_limb_state.commandedqQueue) > 0:
                                    if ((time.time() - self.right_limb_state.lastCommandQueueTime) > TRINAConfig.ur5e_control_rate):
                                        self.right_limb.setConfig(self.right_limb_state.commandedqQueue.pop(0) + [0.0])
                                        self.right_limb_state.lastCommandQueueTime = time.time()
                            elif self.right_limb_state.commandType == 1:
                                if len(self.right_limb_state.commandeddqQueue) > 0:
                                    if ((time.time() - self.right_limb_state.lastCommandQueueTime) > TRINAConfig.ur5e_control_rate):
                                        self.right_limb.setVelocity(self.right_limb_state.commandeddqQueue.pop(0) + [0.0])
                                        self.right_limb_state.lastCommandQueueTime = time.time()
                        else:
                            if not self.right_limb_state.commandSent:
                                ###setting position will clear velocity commands
                                if self.right_limb_state.commandType == 0:
                                    self.right_limb.setConfig(self.right_limb_state.commandedq+[0.0])
                                elif self.right_limb_state.commandType == 1:
                                    self.right_limb.setVelocity(self.right_limb_state.commandeddq + [0.0])
                                self.right_limb_state.commandSent = True

                    ##Base:add set path later
                    self.base.setCommandedVelocity(self.base_state.commandedVel)        
                    ###TODO: add the other components here


            elif self.mode == "Kinematic":
                if self.stopMotionFlag:
                    self.simulated_robot.stopMotion()
                else:
                    if self.simulated_robot.newState():
                        self.left_limb_state.sensedq = self.simulated_robot.getLeftLimbConfig()[0:6]
                        self.left_limb_state.senseddq = self.simulated_robot.getLeftLimbVelocity()[0:6]
                        self.left_limb_state.sensedWrench = []
                        self.right_limb_state.sensedq = self.simulated_robot.getRightLimbConfig()[0:6]
                        self.right_limb_state.senseddq = self.simulated_robot.getRightLimbVelocity()[0:6]
                        self.right_limb_state.sensedWrench = []
                        self.base_state.measuredVel = self.simulated_robot.getBaseVelocity()
                        self.base_state.measuredPos = self.simulated_robot.getBaseConfig()
                        ##Add other components...
                        self.simulated_robot.markRead()

                    ###send commands
                    if self.left_limb_state.commandQueue:
                        if self.left_limb_state.commandType == 0:
                            if len(self.left_limb_state.commandedqQueue) > 0:
                                if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.simulated_robot_control_rate):
                                    self.simulated_robot.setLeftLimbConfig(self.left_limb_state.commandedqQueue.pop(0))
                                    self.left_limb_state.lastCommandQueueTime = time.time()
                        elif self.left_limb_state.commandType == 1:
                            if len(self.left_limb_state.commandeddqQueue) > 0:
                                if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.simulated_robot_control_rate):
                                    self.simulated_robot.setLeftLimbVelocity(self.left_limb_state.commandeddqQueue.pop(0))
                                    self.left_limb_state.lastCommandQueueTime = time.time()
                    else:
                        if not self.left_limb_state.commandSent:
                            ###setting position will clear velocity commands
                            if self.left_limb_state.commandType == 0:
                                self.simulated_robot.setLeftLimbConfig(self.left_limb_state.commandedq)
                            elif self.left_limb_state.commandType == 1:
                                self.simulated_robot.setLeftLimbVelocity(self.left_limb_state.commandeddq)
                            self.left_limb_state.commandSent = True

                    if self.right_limb_state.commandQueue:
                        if self.right_limb_state.commandType == 0:
                            if len(self.right_limb_state.commandedqQueue) > 0:
                                if ((time.time() - self.right_limb_state.lastCommandQueueTime) > TRINAConfig.simulated_robot_control_rate):
                                    self.simulated_robot.setRightLimbConfig(self.right_limb_state.commandedqQueue.pop(0))
                                    self.right_limb_state.lastCommandQueueTime = time.time()
                        elif self.right_limb_state.commandType == 1:
                            if len(self.right_limb_state.commandeddqQueue) > 0:
                                if ((time.time() - self.right_limb_state.lastCommandQueueTime) > TRINAConfig.simulated_robot_control_rate):
                                    self.simulated_robot.setRightLimbVelocity(self.right_limb_state.commandeddqQueue.pop(0))
                                    self.right_limb_state.lastCommandQueueTime = time.time()
                    else:
                        if not self.right_limb_state.commandSent:
                            ###setting position will clear velocity commands
                            if self.right_limb_state.commandType == 0:
                                self.simulated_robot.setRightLimbConfig(self.right_limb_state.commandedq)
                            elif self.right_limb_state.commandType == 1:
                                self.simulated_robot.setRightLimbVelocity(self.right_limb_state.commandeddq)
                            self.right_limb_state.commandSent = True

                    self.simulated_robot.setBaseVelocity(self.base_state.commandedVel) 

            self.controlLoopLock.release()
            
            elapsedTime = time.time() - loopStartTime
            self.t = time.time() - self.startTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    ###TODO
    def setPosition(self,q):
        """q is a list of [left limb, right limb, base] 6+6+3"""
        assert len(q) == 12, "motion.setPosition(): Wrong number of dimensions of config sent"
        self.setLeftLimbPosition(q[0:6])
        self.setRightLimbPosition(q[6:12])
        return 
    def setLeftLimbPosition(self,q):
        """q should be a list of 6 elements"""
        """This will clear the motion queue"""
        assert len(q) == 6, "motion.setLeftLimbPosition(): Wrong number of joint positions sent"
        self.controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandedq = deepcopy(q)
        self.left_limb_state.commandeddq = []
        self.left_limb_state.commandType = 0
        self.left_limb_state.commandQueue = False
        self.left_limb_state.commandedqQueue = []
        self.controlLoopLock.release()
        return 

    def setRightLimbPosition(self,q):
        """q should be a list of 6 elements"""
        assert len(q) == 6, "motion.setLeftLimbPosition(): Wrong number of joint positions sent"
        self.controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandedq = deepcopy(q)
        self.right_limb_state.commandeddq = []
        self.right_limb_state.commandType = 0
        self.right_limb_state.commandQueue = False
        self.right_limb_state.commandedqQueue = []
        self.controlLoopLock.release()
        return 

    def setLeftLimbPositionLinear(self,q,duration):
        """set a motion queue, this will clear the setPosition() commands"""
        assert len(q) == 6, "motion.setLeftLimbPositionLinear(): Wrong number of joint positions sent"
        assert duration > 0, "motion.setLeftLimbPositionLinear(): Duration needs to be a positive number"
        #TODO:add velocity check. Maybe not be able to complete the motion within the duration"
        #Also collision checks
        planningTime = 0.0 + TRINAConfig.ur5e_control_rate
        positionQueue = []
        currentq = self.left_limb_state.sensedq
        difference = vectorops.sub(q,currentq)
        while planningTime < duration:
            positionQueue.append(vectorops.add(currentq,vectorops.mul(difference,planningTime/duration)))
            planningTime = planningTime + TRINAConfig.ur5e_control_rate
        positionQueue.append(q)

        self.controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandType = 0
        self.left_limb_state.commandedqQueue = positionQueue
        self.left_limb_state.commandQueue = True
        self.left_limb_state.commandedq = []
        self.left_limb_state.commandeddq = []
        self.controlLoopLock.release()

    def setRightLimbPositionLinear(self,q,duration):
        """set a motion queue, this will clear the setPosition() commands"""
        assert len(q) == 6, "motion.setRightLimbPositionLinear(): Wrong number of joint positions sent"
        assert duration > 0, "motion.setRightLimbPositionLinear(): Duration needs to be a positive number"
        #TODO:add velocity check. Maybe not be able to complete the motion within the duration"
        #Also collision checks
        planningTime = 0.0 + TRINAConfig.ur5e_control_rate
        positionQueue = []
        currentq = self.right_limb_state.sensedq
        difference = vectorops.sub(q,currentq)
        while planningTime < duration:
            positionQueue.append(vectorops.add(currentq,vectorops.mul(difference,planningTime/duration)))
            planningTime = planningTime + TRINAConfig.ur5e_control_rate
        positionQueue.append(q)
        
        self.controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandType = 0
        self.right_limb_state.commandedqQueue = positionQueue
        self.right_limb_state.commandQueue = True
        self.right_limb_state.commandedq = []
        self.right_limb_state.commandeddq = []
        self.controlLoopLock.release()

    def sensedLeftLimbPosition(self):
        return self.left_limb_state.sensedq

    def sensedRightLimbPosition(self):
        return self.right_limb_state.sensedq

    def setVelocity(self,qdot):
        assert len(qdot) == 12, "motion.setPosition(): Wrong number of dimensions of config sent"
        self.setLeftLimbVelocity(qdot[0:6])
        self.setRightLimbVelcity(qdot[6:12])
        return 

    def setLeftLimbVelocity(self,qdot):
        assert len(qdot) == 6, "motion.setLeftLimbVelocity()): Wrong number of joint velocities sent"
        self.controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandeddq = deepcopy(qdot)
        self.left_limb_state.commandedq = []
        self.left_limb_state.commandType = 1
        self.left_limb_state.commandQueue = False
        self.left_limb_state.commandedqQueue = []
        self.controlLoopLock.release()
        return 

    def setRightLimbVelocity(self,qdot):
        assert len(qdot) == 6, "motion.setRightLimbVelocity()): Wrong number of joint velocities sent"
        self.controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandeddq = deepcopy(qdot)
        self.right_limb_state.commandedq = []
        self.right_limb_state.commandType = 1
        self.right_limb_state.commandQueue = False
        self.right_limb_state.commandedqQueue = []
        self.controlLoopLock.release()
        return 

    def sensedLeftLimbVelocity(self):
        return self.left_limb_state.senseddq()

    def sensedRightLimbVelocity(self):
        return self.right_limb_state.senseddq()

    def setBaseVelocity(self, q):
        assert len(q) == 2 ,"motion.setBaseVelocity(): wrong dimensions"
        self.base_state.commandedVel = deepcopy(q)

    # returns [v, w]
    def sensedBaseVelocity(self):
        return self.base.getMeasuredVelocity()

    # returns [x, y, theta]
    def sensedBasePosition(self):
        return self.base.getPosition()

    def shutdown(self):
        """shutdown the componets... """
        self.shutdownFlag = True
        if self.mode == "Physical":
            if armFlag:
                self.left_limb.stop()
                self.right_limb.stop()
            #stop other components
            self.base.shutdown()
        elif self.mode == "Kinematic":
            self.simulated_robot.shutdown()
        return 0

    def isStarted(self):
        return self.startUp

    def moving(self):
        """Returns true if the robot is currently moving."""
        return self.left_limb.moving() or self.right_limb.moving() or self.base.moving()

    def mode(self):
        return self.mode

    def stopMotion(self):
        """Stops all motion"""
        self.base.stopMotion()
        self.stopMotionFlag = True
        self.stopMotionSent = False
        ##TODO: purge commands

        return
    def resumeMotion(self):
        """The robot is ready to take more commands"""
        self.base.startMotion()
        self.startMotionFlag = False
        return 

    def mirror_arm_config(self,config):
        """given the Klampt config of the left or right arm, return the other"""
        RConfig = []
        RConfig.append(-config[0])
        RConfig.append(-config[1]-math.pi)
        RConfig.append(-config[2])
        RConfig.append(-config[3]+math.pi)
        RConfig.append(-config[4])
        RConfig.append(-config[5])

        for ele in RConfig:
            if ele >= 2*math.pi or ele <= -2*math.pi:
                print('out of range..')
                return []
        return RConfig

    def getWorld(self):
        return self.simulated_robot.getWorld()

if __name__=="__main__":

    server = SimpleWebSocketServer('130.126.139.236', 1234, SimpleEcho)
    server.serveforever()