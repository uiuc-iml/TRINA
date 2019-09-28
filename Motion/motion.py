import os
import sys
import time
from threading import Thread, Lock
import threading
import limbController
import TRINAConfig #network configs and other configs
import motionStates #state structures
import utils 
from copy import deepcopy
from klampt.math import vectorops 
class Motion:
    def __init__(self):
        ##T#he individual components
        self.left_limb = LimbController(TRINAConfig.left_limb_address)
        self.right_limb = LimbController(TRINAConfig.right_limb_address)
	    #self.base = MobileBase()
        self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        self.currentGravityVector = [0,0,-9.81]  ##expressed in the robot base local frame, with x pointint forward and z up
        ##TODO: Add other components

        ###Other 
        self.startTime = time.time()
        self.t = 0 #time since startup
        self.startUp = False
        self.dt = 0.002
        self.automaticMode = False #if in this mode, a planner or recorded motion can take over. Can be interruped by stopMotion()
        self.stopMotionFlag = False
        self.stopMotionSent = False
        self.shutdownFlag = False
        self.controlLoopLock = Lock()

    def time(self):
        """Returns the time since robot startup, in s"""
        return self.t

    def startup(self):
        """After starting, all components stay where they are and update their positions immediately"""
        ###start limb
        #TODO: when we start the limbs, we need to make sure that the gravity vectors are set right....

        ### The torso controller need to be started first to read the current torso config..


        #assume this is the tilt angle 
        tiltAngle = 0.0
        self.currentGravityVector = [0,0,-9.81]

        res = self.left_limb.start()
        if res == False:
            #better to replace this with logger
            print("motion.startup(): ERROR, left limb start failure.")
            return False
        else:
            print("motion.startup(): left limb started.")

        res = self.right_limb.start()    
        if res == False:
            #better to replace this with logger
            print("motion.startup(): ERROR, left limb start failure.")
            return False
        else:
            print("motion.startup(): left limb started.")


        #start the other components        

        #overrides the default Ctrl+C behavior which kills the program
        #check this....
        #def interrupter(x,y):
        #    self.shutdown()
        #    raise KeyboardInterrupt()
        #signal.signal(signal.SIGINT,interrupter)
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

            if self.stopMotionFlag:
                if not self.stopMotionSent: #send only once to avoid drifting...
                    self.left_limb.stopMotion()
                    self.right_limb.stopMotion()
                    self.stopMotionSent = True
            else:
                ###update current state
                if left_limb.newState():
                    self.left_limb_state.sensedq = left_limb.getConfig()[0:6]
                    self.left_limb_state.senseddq = left_limb.getVelocity()[0:6]
                    self.left_limb_state.sensedWrench = left_limb.getWrench()
                    left_limb.markRead()
                if right_limb.newState():
                    self.right_limb_state.sensedq = left_limb.getConfig()[0:6]
                    self.right_limb_state.senseddq = left_limb.getVelocity()[0:6]
                    self.right_limb_state.sensedWrench = left_limb.getWrench()
                    right_limb.markRead()

                ###send commands
                if left_limb_state.commandQueue:
                    if self.left_limb_state.commandType == 0:
                        if len(left_limb_state.commandedqQueue) > 0:
                            if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.ur5eControlRate):
                                self.left_limb.setConfig(left_limb_state.commandedqQueue.pop(0) + [0.0])
                                self.left_limb_state.lastCommandQueueTime = time.time()
                    elif self.left_limb_state.commandType == 1:
                        if len(left_limb_state.commandeddqQueue) > 0:
                            if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.ur5eControlRate):
                                self.left_limb.setVelocity(left_limb_state.commandeddqQueue.pop(0) + [0.0])
                                self.left_limb_state.lastCommandQueueTime = time.time()
                else:
                    if not self.left_limb_state.commandSent:
                        ###setting position will clear velocity commands
                        if self.left_limb_state.commandType == 0:
                            self.left_limb.setConfig(self.left_limb_state.commandedq+[0.0])
                        elif self.left_limb_state.commandType == 1:
                            self.left_limb.setVelocity(self.left_limb_state.commandeddq + [0.0])
                        self.left_limb_state.commandSent = True

                if right_limb_state.commandQueue:
                    if self.right_limb_state.commandType == 0:
                        if len(right_limb_state.commandedqQueue) > 0:
                            if ((time.time() - self.right_limb_state.lastCommandQueueTime) > TRINAConfig.ur5eControlRate):
                                self.right_limb.setConfig(right_limb_state.commandedqQueue.pop(0) + [0.0])
                                self.right_limb_state.lastCommandQueueTime = time.time()
                    elif self.right_limb_state.commandType == 1:
                        if len(right_limb_state.commandeddqQueue) > 0:
                            if ((time.time() - self.right_limb_state.lastCommandQueueTime) > TRINAConfig.ur5eControlRate):
                                self.right_limb.setVelocity(right_limb_state.commandeddqQueue.pop(0) + [0.0])
                                self.right_limb_state.lastCommandQueueTime = time.time()
                else:
                    if not self.right_limb_state.commandSent:
                        ###setting position will clear velocity commands
                        if self.right_limb_state.commandType == 0:
                            self.right_limb.setConfig(self.right_limb_state.commandedq+[0.0])
                        elif self.right_limb_state.commandType == 1:
                            self.right_limb.setVelocity(self.right_limb_state.commandeddq + [0.0])
                        self.right_limb_state.commandSent = True
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
        planningTime = 0.0 + TRINAConfig.ur5eControlRate
        positionQueue = []
        currentq = self.left_limb_state.sensedq
        difference = vectorops.sub(q,currentq)
        while planningTime < duration:
            positionQueue.append(vectorops.add(currentq,vectorops.mul(difference,planningTime/duration)))
            planningTime = planningTime + TRINAConfig.ur5eControlRate
        positionQueue.append(q)

        self.controlLoopLock.acquire()
        self.left_limb_state.commendSent = False
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
        planningTime = 0.0 + TRINAConfig.ur5eControlRate
        positionQueue = []
        currentq = self.right_limb_state.sensedq
        difference = vectorops.sub(q,currentq)
        while planningTime < duration:
            positionQueue.append(vectorops.add(currentq,vectorops.mul(difference,planningTime/duration)))
            planningTime = planningTime + TRINAConfig.ur5eControlRate
        positionQueue.append(q)
        
        self.controlLoopLock.acquire()
        self.right_limb_state.commendSent = False
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

    def setLeftLimbVelocity(self,qdot)
        assert len(qdot) == 6, "motion.setLeftLimbVelocity()): Wrong number of joint velocities sent"
        self.controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandeddq = deepcopy(qdot)
        self.left_limb_state.commandedq = []
        self.left_limb_state.commandType = 1
        self.left_limb_state.commandQueue = False
        self.left_limb_state.commandedqQueue = []
        return 

    def setRightLimbVelocity(self,qdot)
        assert len(qdot) == 6, "motion.setRightLimbVelocity()): Wrong number of joint velocities sent"
        self.controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandeddq = deepcopy(qdot)
        self.right_limb_state.commandedq = []
        self.right_limb_state.commandType = 1
        self.right_limb_state.commandQueue = False
        self.right_limb_state.commandedqQueue = []
        return 

    def sensedLeftLimbVelocity(self):
        return self.left_limb_state.senseddq()

    def sensedRightLimbVelocity(self):
        return self.right_limb_state.senseddq()



    def shutdown(self):
        """shutdown the componets... """
        left_limb.stop()
        right_limb.stop()
        #stop other components
        return 0

    def isStarted(self):
        return self.startUp

    def moving(self):
        """Returns true if the robot is currently moving."""
        return self.left_limb.moving() or self.right_limb.moving()

    def stopMotion(self):
        """Stops all motion"""
        self.stopMotionFlag = True
        self.stopMotionSent = False
        ##TODO: purge commands

        return
    def resumeMotion(self):
        """The robot is ready to take more commands"""
        self.startMotionFlag = False
        return 

    def tuckArm(self,arm='left'):
        """While tucking arms, the other components are not allowed to move"""

        return



if __name__=="__main__":


    print "Testing TRINA2 Motion Module..."
    robot = motion()
    if not robot.isStarted():
        res = robot.startup()
        if not res:
            raise RuntimeError("Ebolabot Motion could not be started")
    else:
        print "Robot started by another process"
        
    print "Is robot started?",robot.isStarted()
    print "Left limb:"
    print "   configuration:",robot.left_limb.sensedPosition()
    print "   velocity:",robot.left_limb.sensedVelocity()
    print "   effort:",robot.left_limb.sensedEffort()
    print "   motion queue enabled:",robot.left_mq.enabled()
    print "   motion queue moving:",robot.left_mq.moving()
    print "Right limb:"
    print "   configuration:",robot.right_limb.sensedPosition()
    print "   velocity:",robot.right_limb.sensedVelocity()
    print "   effort:",robot.right_limb.sensedEffort()
    print "   motion queue enabled:",robot.right_mq.enabled()
    print "   motion queue moving:",robot.right_mq.moving()
    print "Left gripper:"
    print "   end effector xform:",robot.left_ee.sensedTransform()
    print "   type:",robot.left_gripper.type()
    print "   enabled:",robot.left_gripper.enabled()
    print "   moving:",robot.left_gripper.moving()
    print "   position:",robot.left_gripper.position()
    print "Right gripper:"
    print "   end effector xform:",robot.right_ee.sensedTransform()
    print "   type:",robot.right_gripper.type()
    print "   enabled:",robot.right_gripper.enabled()
    print "   moving:",robot.right_gripper.moving()
    print "   position:",robot.right_gripper.position()
    print
    print "Shutting down..."
    robot.shutdown()
    print "Shutdown completed"
