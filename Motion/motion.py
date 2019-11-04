import os
import signal
import sys
import time
import math
from threading import Thread, Lock
import threading
from limbController import LimbController
from baseController import BaseController
from gripperController import GripperController
from kinematicController import KinematicController
from torsoController import TorsoController
import TRINAConfig #network configs and other configs
from motionStates import * #state structures
#import convenienceFunctions
from copy import deepcopy
from klampt.math import vectorops,so3
from klampt import vis
from klampt.model import ik, collide
import numpy as np
from klampt import WorldModel
##Two different modes:
#"Physical" == actual robot
#"Kinematic" == Klampt model

import os
dirname = os.path.dirname(__file__)
#getting absolute model name
model_name = os.path.join(dirname, "data/TRINA_world_reflex.xml")

armFlag = True
class Motion:

    def __init__(self,  mode = 'Kinematic', model_path = model_name):
        self.mode = mode
        self.model_path = model_path
        self.computation_model_path = "data/TRINA_world.xml"
        if self.mode == "Kinematic":
            print("initiating Kinematic controller")
            self.simulated_robot = KinematicController(model_path)
            print("initiated Kinematic controller")
        elif self.mode == "Physical":
            ##The individual components
            if armFlag:
                self.left_limb = LimbController(TRINAConfig.left_limb_address,gripper=False,gravity = TRINAConfig.left_limb_gravity_upright)
                self.right_limb = LimbController(TRINAConfig.right_limb_address,gripper=False,gravity = TRINAConfig.right_limb_gravity_upright)
            self.base = BaseController()
            self.left_gripper = GripperController()
            self.currentGravityVector = [0,0,-9.81]  ##expressed in the robot base local frame, with x pointint forward and z up
            self.torso = TorsoController()
            ##TODO: Add other components
        else:
            raise RuntimeError('Wrong mode specified')
        self.world = WorldModel()
        res = self.world.readFile(self.computation_model_path)
        if not res:
            raise RuntimeError("unable to load model")
        self.collider = collide.WorldCollider(self.world)
        self.robot_model = self.world.robot(0)
        self.left_EE_link = self.robot_model.link(16)
        self.left_active_Dofs = [10,11,12,13,14,15]
        self.right_EE_link = self.robot_model.link(33)
        self.right_active_Dofs = [27,28,29,30,31,32]

        self.left_limb_state = LimbState()
        self.right_limb_state = LimbState()
        self.base_state = BaseState()
        self.torso_state = TorsoState()
        self.left_gripper_state = GripperState()
        self.startTime = time.time()
        self.t = 0 #time since startup
        self.startUp = False
        self.dt = 0.002
        self.automatic_mode = False #if in this mode, a planner or recorded motion can take over. Can be interruped by stopMotion()
        self.stop_motion_flag = False
        self.stop_motion_sent = False
        self.shut_down_flag = False
        self._controlLoopLock = Lock()
        self.cartedian_drive_failure = False
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
                    print("motion.startup(): ERROR, right limb start failure.")
                    return False
                else:
                    print("motion.startup(): right limb started.")
                    self.right_limb_state.sensedq = self.right_limb.getConfig()[0:6]
                    self.right_limb_state.senseddq = self.right_limb.getVelocity()[0:6]
                    self.right_limb_state.sensedWrench = self.right_limb.getWrench()
            # start the other components
            self.base.start()
            self.left_gripper.start()
            # TODO: add more components...

        controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()
        print("motion.startup():robot started")
        self.startUp = True
        return self.startUp

    def _controlLoop(self):
        """main control thread, synchronizing all components"""
        """in each loop,states are updated and new commands are issued"""
        self.robot_start_time = time.time()
        print("motion.controlLoop(): controlLoop started.")
        while not self.shut_down_flag:
            loopStartTime = time.time()
            self.t = time.time() - self.startTime
            ###lock the thread
            self._controlLoopLock.acquire()
            if self.mode == "Physical":
                if self.stop_motion_flag:
                    if not self.stop_motion_sent: #send only once to avoid drifting...
                        if armFlag:
                            self.left_limb.stopMotion()
                            self.right_limb.stopMotion()
                        self.base.stopMotion()
                        self.left_gripper.stop()
                        self.stop_motion_sent = True #unused
                else:
                    ###update current state
                    if self.base.newState():
                        self.base_state.measuredVel = self.base.getMeasuredVelocity()
                        self.base_state.measuredPos = self.base.getPosition()
                        self.base.markRead()

                    if self.torso.newState():
                        tilt, height, _, _ = self.torso.getStates()
                        self.torso_state.measuredTilt = tilt
                        self.torso_state.measuredHeight = height
                        self.torso.markRead()

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

                    if self.left_gripper.new_state():
                       self.left_gripper_state.sense_finger_set = self.left_gripper.sense_finger_set
                       self.left_gripper.mark_read()
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
                    if self.base_state.commandType == 1:
                        self.base.setCommandedVelocity(self.base_state.commandedVel)
                    elif self.base_state.commandType == 0 and not base_state.commandSent:
                        self.base_state.commandSent = True
                        self.base.setTargetPosition(self.base_state.commandedVel)

                    if not self.torso_state.commandSent:
                        self.torso_state.commandSent = True
                        self.torso.setTargetPositions(self.torso_state.commandedHeight, self.torso_state.commandedTilt)

                    if self.left_gripper_state.commandType == 0:
                       self.left_gripper.setPose(self.left_gripper_state.command_finger_set)
                    elif self.left_gripper_state.commandType == 1:
                       self.left_gripper.setVelocity(self.left_gripper_state.command_finger_set)

                    ###TODO: add the other components here

                    ##update internal robot model, does not use the base's position and orientation
                    ##basically assumes that the world frame is the frame centered at the base local frame, on the floor.
                    #robot_modelQ = self.base_state.sensedq + [0]*7 +self.left_limb_state.sensedq+[0]*11+self.right_limb_state.sensedq+[0]*10
                    robot_model_Q = [0]*3 + [0]*7 +self.left_limb_state.sensedq+[0]*11+self.right_limb_state.sensedq+[0]*10
                    self.robot_model.setConfig(robot_model_Q)

            elif self.mode == "Kinematic":
                if self.stop_motion_flag:
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
                        self.left_gripper_state.sense_finger_set = self.simulated_robot.getLeftGripperPosition()
                        ##Add other components...
                        self.simulated_robot.markRead()

                    ###send commands
                    if self.left_limb_state.commandQueue:
                        if self.left_limb_state.commandType == 0:
                            if len(self.left_limb_state.commandedqQueue) > 0:
                                if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.simulated_robot_control_rate):
                                    tmp = self.left_limb_state.commandedqQueue.pop(0)
                                    self.simulated_robot.setLeftLimbConfig(tmp)
                                    self.left_limb_state.lastCommandQueueTime = time.time()
                        elif self.left_limb_state.commandType == 1:
                            if len(self.left_limb_state.commandeddqQueue) > 0:
                                if ((time.time() - self.left_limb_state.lastCommandQueueTime) > TRINAConfig.simulated_robot_control_rate):
                                    self.simulated_robot.setLeftLimbVelocity(self.left_limb_state.commandeddqQueue.pop(0))
                                    self.left_limb_state.lastCommandQueueTime = time.time()
                    #### cartesian drive mode
                    ####
                    elif self.left_limb_state.cartesianDrive:
                        #clock1 = time.time()
                        flag = 1
                        while flag:
                            res, target_config = self._left_limb_cartesian_drive(self.left_limb_state.driveTransform)
                            if res == 0:
                                #set to position mode...
                                self.cartesian_drive_failure = True
                                self.left_limb_state.commandSent = False
                                self.left_limb_state.commandedq = deepcopy(self.sensedLeftLimbPosition())
                                self.left_limb_state.commandeddq = []
                                self.left_limb_state.commandType = 0
                                self.left_limb_state.commandQueue = False
                                self.left_limb_state.commandedqQueue = []
                                self.left_limb_state.cartesianDrive = False
                                break
                            elif res == 1:
                                flag = 1
                            elif res == 2:
                                flag = 0
                                self.simulated_robot.setLeftLimbConfig(target_config)

                            #print(res)
                        #print("CartesianDrive IK took",time.time() - clock1, "secs")
                    ####                           

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

                    elif self.right_limb_state.cartesianDrive:
                        #clock1 = time.time()
                        flag = 1
                        while flag:
                            res, target_config = self._right_limb_cartesian_drive(self.right_limb_state.driveTransform)
                            if res == 0:
                                #set to position mode...
                                self.cartesian_drive_failure = True
                                self.right_limb_state.commandSent = False
                                self.right_limb_state.commandedq = deepcopy(self.sensedRightLimbPosition())
                                self.right_limb_state.commandeddq = []
                                self.right_limb_state.commandType = 0
                                self.right_limb_state.commandQueue = False
                                self.right_limb_state.commandedqQueue = []
                                self.right_limb_state.cartesianDrive = False
                                break
                            elif res == 1:
                                flag = 1
                            elif res == 2:
                                flag = 0
                                self.simulated_robot.setRightLimbConfig(target_config)              
                    else:
                        if not self.right_limb_state.commandSent:
                            ###setting position will clear velocity commands
                            if self.right_limb_state.commandType == 0:
                                self.simulated_robot.setRightLimbConfig(self.right_limb_state.commandedq)
                            elif self.right_limb_state.commandType == 1:
                                self.simulated_robot.setRightLimbVelocity(self.right_limb_state.commandeddq)
                            self.right_limb_state.commandSent = True

                    if self.base_state.commandType == 1:
                        self.simulated_robot.setBaseVelocity(self.base_state.commandedVel)
                    elif self.base_state.commandType == 0 and not base_state.commandSent:
                        base_state.commandSent = True
                        self.base.setTargetPosition(self.base_state.commandedVel)

                    ##gripper
                    self.simulated_robot.setLeftGripperPosition(self.left_gripper_state.command_finger_set)
                    robot_model_Q = [0]*3 + [0]*7 +self.left_limb_state.sensedq+[0]*11+self.right_limb_state.sensedq+[0]*10
                    self.robot_model.setConfig(robot_model_Q)
            

            elapsedTime = time.time() - loopStartTime
            self.t = time.time() - self.startTime
            if elapsedTime < self.dt:
                #print("before sleep",time.time() - self.robot_start_time)        
                time.sleep(self.dt-elapsedTime)
                #print("after sleep", time.time() - self.robot_start_time)
            else:
                pass
            self._controlLoopLock.release()
            #print("main",time.time() - robotStartTime)
        print("motion.controlThread: exited")
    ###TODO
    def setPosition(self,q):
        """set the position of the entire robot"""
        """under development rn"""
        """q is a list of [left limb, right limb, base] 6+6+3"""
        assert len(q) == 12, "motion.setPosition(): Wrong number of dimensions of config sent"
        self.setLeftLimbPosition(q[0:6])
        self.setRightLimbPosition(q[6:12])
        return
    def setLeftLimbPosition(self,q):
        """set the left limb joint position, and the robot moves as fast as possible"""
        """q should be a list of 6 elements"""
        """This will clear the motion queue"""
        assert len(q) == 6, "motion.setLeftLimbPosition(): Wrong number of joint positions sent"
        self._controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandedq = deepcopy(q)
        self.left_limb_state.commandeddq = []
        self.left_limb_state.commandType = 0
        self.left_limb_state.commandQueue = False
        self.left_limb_state.commandedqQueue = []
        self.left_limb_state.cartesianDrive = False
        self._controlLoopLock.release()
        return

    def setRightLimbPosition(self,q):
        """set the right limb joint sposition, and the robot moves as fast as possible"""
        """q should be a list of 6 elements"""
        assert len(q) == 6, "motion.setLeftLimbPosition(): Wrong number of joint positions sent"
        self._controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandedq = deepcopy(q)
        self.right_limb_state.commandeddq = []
        self.right_limb_state.commandType = 0
        self.right_limb_state.commandQueue = False
        self.right_limb_state.commandedqQueue = []
        self.right_limb_state.cartesianDrive = False
        self._controlLoopLock.release()
        return

    def setLeftLimbPositionLinear(self,q,duration):
        """left limb moves to q in duration time"""
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

        #print("motion.setLeftLimbPositionLinear")
        self._controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandType = 0
        self.left_limb_state.commandedqQueue = positionQueue
        self.left_limb_state.commandQueue = True
        self.left_limb_state.commandedq = []
        self.left_limb_state.commandeddq = []
        self.left_limb_state.cartesianDrive = False
        self._controlLoopLock.release()

    def setRightLimbPositionLinear(self,q,duration):
        """limb moves to q in duration time"""
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

        self._controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandType = 0
        self.right_limb_state.commandedqQueue = positionQueue
        self.right_limb_state.commandQueue = True
        self.right_limb_state.commandedq = []
        self.right_limb_state.commandeddq = []
        self.right_limb_state.cartesianDrive = False
        self._controlLoopLock.release()
        return

    def sensedLeftLimbPosition(self):
        return self.left_limb_state.sensedq

    def sensedRightLimbPosition(self):
        return self.right_limb_state.sensedq

    def setVelocity(self,qdot):
        """set the velocity of the entire robot, under development rn"""
        assert len(qdot) == 12, "motion.setPosition(): Wrong number of dimensions of config sent"
        self.setLeftLimbVelocity(qdot[0:6])
        self.setRightLimbVelcity(qdot[6:12])
        return

    def setLeftLimbVelocity(self,qdot):
        """set limb joint velocities"""
        assert len(qdot) == 6, "motion.setLeftLimbVelocity()): Wrong number of joint velocities sent"
        self._controlLoopLock.acquire()
        self.left_limb_state.commandSent = False
        self.left_limb_state.commandeddq = deepcopy(qdot)
        self.left_limb_state.commandedq = []
        self.left_limb_state.commandType = 1
        self.left_limb_state.commandQueue = False
        self.left_limb_state.commandedqQueue = []
        self.left_limb_state.cartesianDrive = False
        self._controlLoopLock.release()
        return

    def setRightLimbVelocity(self,qdot):
        """set limb joint velocities"""
        assert len(qdot) == 6, "motion.setRightLimbVelocity()): Wrong number of joint velocities sent"
        self._controlLoopLock.acquire()
        self.right_limb_state.commandSent = False
        self.right_limb_state.commandeddq = deepcopy(qdot)
        self.right_limb_state.commandedq = []
        self.right_limb_state.commandType = 1
        self.right_limb_state.commandQueue = False
        self.right_limb_state.commandedqQueue = []
        self.right_limb_state.cartesianDrive = False
        self._controlLoopLock.release()
        return

    def setLeftEEInertialTransform(self,Ttarget,duration):
        """Set the trasform of the arm w.r.t. the base frame. Assmume that the torso are not moving"""
        #print("motion.setLeftEEInertialTransform():started..")
        self._controlLoopLock.acquire()
        initial = self.robot_model.getConfig()
        goal = ik.objective(self.left_EE_link,R=Ttarget[0],t = Ttarget[1])
        if ik.solve_nearby(goal,maxDeviation=3,activeDofs = self.left_active_Dofs):
            target_config = self.robot_model.getConfig()
            print("motion.setLeftEEInertialTransform():IK solve successful")
        else:
            print('motion.setLeftEEInertialtransform():IK solve failure: no IK solution found')
            return
        res = self._check_collision_linear(self.robot_model,initial,target_config,15)
        #print(res)
        if res:
            print('motion.setLeftEEInertialtransform():Self-collision midway')
            return
        else:
            print("motion.setLeftEEInertialTransform():No collisoin")

        self.robot_model.setConfig(initial)
        self._controlLoopLock.release()
        self.setLeftLimbPositionLinear(target_config[10:16],duration)

        return

    def setLeftEEVelocity(self,v = None,w = None, tool = [0,0,0]):
        """set the ee to translate at v and rotate at w for a specific amount of duration of time"""
        """implemented using position control"""
        """collision detection not implemented rn..."""
        self._controlLoopLock.acquire()
        self.left_limb_state.commandedq = []
        self.left_limb_state.commandedqQueue = []
        self.left_limb_state.commandeddq = []
        self.left_limb_state.commandQueue = False
        self.cartesian_drive_failure = False
        ##cartesian velocity drive
        if v:
            self.left_limb_state.cartesianDriveV = deepcopy(v)
        if w:
            self.left_limb_state.cartesianDriveW = deepcopy(w)
        if v and w:
            self.left_limb_state.cartesianMode = 0
        else:
            if v:
                self.left_limb_state.cartesianMode = 1
            elif w:
                #self.left_limb_state.cartesianMode = 2
                print("motion.setLeftEEVelocity(): wrong input, can't specify w alone")
            else:
                print("motion.setLeftEEVelocity(): wrong input")
        self.left_limb_state.cartesianDrive = True
        (R,t) = self.left_EE_link.getTransform()
        self.left_limb_state.startTransform = (R,vectorops.add(so3.apply(R,tool),t))
        self.left_limb_state.driveTransform = (R,vectorops.add(so3.apply(R,tool),t))
        self.left_limb_state.driveSpeedAdjustment = 1.0
        self.left_limb_state.toolCenter = deepcopy(tool)
        self._controlLoopLock.release()

    def setRightEEInertialTransform(self,Ttarget,duration):
        """Set the trasform of the arm w.r.t. the base frame. Assmume that the torso are not moving"""
        #print("motion.setLeftEEInertialTransform():started..")
        self._controlLoopLock.acquire()
        initial = self.robot_model.getConfig()
        goal = ik.objective(self.right_EE_link,R=Ttarget[0],t = Ttarget[1])
        if ik.solve_nearby(goal,maxDeviation=3,activeDofs = self.right_active_Dofs):
            target_config = self.robot_model.getConfig()
            print("motion.setRightEEInertialTransform():IK solve successful")
        else:
            print('motion.setRightEEInertialtransform():IK solve failure: no IK solution found')
            return
        res = self._check_collision_linear(self.robot_model,initial,target_config,15)
        #print(res)
        if res:
            print('motion.setRighttEEInertialtransform():Self-collision midway')
            return
        else:
            print("motion.setRightEEInertialTransform():No collisoin")

        self.robot_model.setConfig(initial)
        self._controlLoopLock.release()
        self.setRightLimbPositionLinear(target_config[27:33],duration)

        return

    def setRightEEVelocity(self,v = None,w = None, tool = [0,0,0]):
        """set the ee to translate at v and rotate at w for a specific amount of duration of time"""
        """implemented using position control"""
        """collision detection not implemented rn..."""
        self._controlLoopLock.acquire()
        self.right_limb_state.commandedq = []
        self.right_limb_state.commandedqQueue = []
        self.right_limb_state.commandeddq = []
        self.right_limb_state.commandQueue = False
        self.cartesian_drive_failure = False
        ##cartesian velocity drive
        if v:
            self.right_limb_state.cartesianDriveV = deepcopy(v)
        if w:
            self.right_limb_state.cartesianDriveW = deepcopy(w)
        if v and w:
            self.right_limb_state.cartesianMode = 0
        else:
            if v:
                self.right_limb_state.cartesianMode = 1
            elif w:
                #self.right_limb_state.cartesianMode = 2
                print("motion.setRightEEVelocity(): wrong input, can't specify w alone")
            else:
                print("motion.setRightEEVelocity(): wrong input")
        self.right_limb_state.cartesianDrive = True
        (R,t) = self.right_EE_link.getTransform()
        self.right_limb_state.startTransform = (R,vectorops.add(so3.apply(R,tool),t))
        self.right_limb_state.driveTransform = (R,vectorops.add(so3.apply(R,tool),t))
        self.right_limb_state.driveSpeedAdjustment = 1.0
        self.right_limb_state.toolCenter = deepcopy(tool)
        self._controlLoopLock.release()


    def sensedLeftEETransform(self):
        """Return the transform w.r.t. the base frame"""
        return self.left_EE_link.getTransform()

    def sensedRightEETransform(self):
        """Return the transform w.r.t. the base frame"""
        return self.right_EE_link.getTransform()


    def sensedLeftLimbVelocity(self):
        return self.left_limb_state.senseddq()

    def sensedRightLimbVelocity(self):
        return self.right_limb_state.senseddq()

    def setBaseTargetPosition(self, q, vel):
        """set the local target position of the base"""
        """base constructs a path to go to the desired position"""
        assert len(q) == 3, "motion.setBaseTargetPosition(): wrong dimensions"
        self._controlLoopLock.acquire()
        self.base_state.commandType = 0
        self.base_state.commandedTargetPosition = deepcopy(q)
        self.base_state.pathFollowingVel = vel
        self.base_state.commandSent = False
        self._controlLoopLock.release()

    def setBaseVelocity(self, q):
        """set the velocity of the base relative to the local base frame"""
        assert len(q) == 2 ,"motion.setBaseVelocity(): wrong dimensions"
        self._controlLoopLock.acquire()
        self.base_state.commandType = 1
        self.base_state.commandedVel = deepcopy(q)
        self.base_state.commandSent = False
        self._controlLoopLock.release()
    def setTorsoTargetPosition(self, q):
        assert len(q) == 2, "mtion.SetTorsoTargetPosition(): wrong dimensions"
        height, tilt = q
        self._controlLoopLock.acquire()
        self.torso_state.commandedHeight = height
        self.torso_state.commandedTilt = tilt
        self.torso_state.commandSent = False
        self._controlLoopLock.release()

    # returns [v, w]
    def sensedBaseVelocity(self):
        return self.base_state.measuredVel

    # returns [x, y, theta]
    def sensedBasePosition(self):
        return self.base_state.measuredPos

    # returns [height, tilt]
    def sensedTorsoPosition(self):
        return [self.torso_state.measuredHeight, self.torso_state.measuredTilt]

    def setGripperPosition(self, position):
        self._controlLoopLock.acquire()
        self.left_gripper_state.commandType = 0
        self.left_gripper_state.command_finger_set = deepcopy(position)
        self._controlLoopLock.release()
    def setGripperVelocity(self,velocity):
        self.left_gripper_state.commandType = 1
        self.left_gripper_state.command_finger_set = deepcopy(velocity)

    def sensedGripperPosition(self):
        return self.left_gripper_state.sense_finger_set

    def getKlamptCommandedPosition(self):
        ###using attached grippers as reflex grippers ###

        if self.left_limb_state.commandedq and self.right_limb_state.commandedq:
            return [0]*10 + self.left_limb_state.commandedq + [0]*19 + self.right_limb_state.commandedq + [0]*18
        else:
            return self.getKlamptSensedPosition()

    def getKlamptSensedPosition(self):
        return [0]*10 + self.left_limb_state.sensedq + [0]*19 + self.right_limb_state.sensedq + [0]*18

    def shutdown(self):
        """shutdown the componets... """
        self.shut_down_flag = True
        if self.mode == "Physical":
            if armFlag:
                self.left_limb.stop()
                self.right_limb.stop()
            #stop other components
            self.base.shutdown()
            self.left_gripper.shutDown()
            self.torso.shutdown()
        elif self.mode == "Kinematic":
            self.simulated_robot.shutdown()
        return 0

    def isStarted(self):
        return self.startUp

    def moving(self):
        """Returns true if the robot is currently moving."""
        return self.left_limb.moving() or self.right_limb.moving() or self.base.moving() or self.left_gripper.moving() or self.torso.moving()

    def mode(self):
        return self.mode

    def stopMotion(self):
        """Stops all motion"""
        self.base.stopMotion()
        self.left_gripper.stop()
        self.stop_motion_flag = True
        self.stop_motion_sent = False
        ##TODO: purge commands

        return
    def resumeMotion(self):
        """The robot is ready to take more commands"""
        self.base.startMotion()
        self.startMotionFlag = False
        self.left_gripper.resume()
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

    def cartesianDriveFail(self):
        return self.cartesian_drive_failure

    ###Below are internal helper functions
    def _check_collision_linear(self,robot,q1,q2,disrectization):
        #print('_check_collision_linear():started')
        lin = np.linspace(0,1,disrectization)
        initialConfig = robot.getConfig()
        diff = vectorops.sub(q2,q1)
        counter = 0
        for c in lin:
            #print('_check_collision_linear():started',c)
            Q=vectorops.madd(q1,diff,c)
            robot.setConfig(Q)
            collisions = self.collider.robotSelfCollisions(robot)
            colCounter = 0
            for col in collisions:
                colCounter = colCounter + 1
            if colCounter > 0:
                return True
            #add_ghosts(robot,vis,[robot.getConfig()],counter)
            counter = counter + 1
        robot.setConfig(initialConfig)
        #print('_check_collision_linear():no collision')
        return False

    def _limit_arm_position(self,config):
        modified = []
        for i in range(6):
            if config[i] > TRINAConfig.limb_position_upper_limits[i]:
                modified.append(TRINAConfig.limb_position_upper_limits[i])
            elif config[i] < TRINAConfig.limb_position_lower_limits[i]:
                modified.append(TRINAConfig.limb_position_lower_limits[i])
            else:
                modified.append(config[i])
        return modified

    def _arm_is_in_limit(self,config,upper,lower):
        for [q,qu,ql] in zip(config,upper,lower):
            if q > qu or q < ql:
                return False

        return True

    def _left_limb_cartesian_drive(self,current_transform):
        v = self.left_limb_state.cartesianDriveV
        w = self.left_limb_state.cartesianDriveW
        amount = self.dt * self.left_limb_state.driveSpeedAdjustment
        #print("Before:",self.left_limb_state.driveTransform)
        #print(v,amount,vectorops.mul(v,amount))
        target_transform = (so3.mul(so3.from_moment(vectorops.mul(w,amount)),\
            self.left_limb_state.driveTransform[0]),vectorops.add(\
            self.left_limb_state.driveTransform[1],vectorops.mul(v,amount)))
        #print("After:",self.left_limb_state.driveTransform)
        #joint position limits from the joint speed limit
        joint_upper_limits = vectorops.add(self.left_limb_state.sensedq,vectorops.mul(\
            TRINAConfig.limb_velocity_limits,self.dt))
        joint_lower_limits = vectorops.add(self.left_limb_state.sensedq,vectorops.mul(\
            TRINAConfig.limb_velocity_limits,-self.dt))
        if self.left_limb_state.cartesianMode == 0:
            goal = ik.objective(self.left_EE_link,R=target_transform[0],\
                t = vectorops.sub(target_transform[1],so3.apply(target_transform[0],self.left_limb_state.toolCenter)))
        elif self.left_limb_state.cartesianMode == 1:
            goal = ik.objective(self.left_EE_link,local = [0,0,0], \
                world = vectorops.sub(target_transform[1],so3.apply(target_transform[0],self.left_limb_state.toolCenter)))
        #elif self.left_limb_state.cartesianMode == 2:
        #                goal = ik.objective(self.left_EE_link,R=target_transform[0])
        
        initialConfig = self.robot_model.getConfig()
        res = ik.solve_nearby(goal,maxDeviation=0.5,activeDofs = self.left_active_Dofs,tol=0.000001)
        failFlag = False
        if res:
            if self._arm_is_in_limit(self.robot_model.getConfig()[10:16],joint_upper_limits,joint_lower_limits):
                pass
            else:
                failFlag = True
                #print(self.left_limb_state.driveSpeedAdjustment)
                #print("motion.controlLoop():IK not in joint limit")
                #print(self.left_limb_state.sensedq)
                #print(self.robot_model.getConfig()[10:16])
                #print(target_transform)
        else:
            failFlag = True
            #print("motion.controlLoop():IK solution not found")
        if failFlag:
            self.left_limb_state.driveSpeedAdjustment = self.left_limb_state.driveSpeedAdjustment - 0.1
            if self.left_limb_state.driveSpeedAdjustment < 0.001:
                self.left_limb_state.cartesianDrive = False
                print("motion.controlLoop():CartesianDrive IK has failed completely,exited..")
                return 0,0 # 0 means the IK has failed completely
            else:
                #print("motion.controlLoop():CartesianDrive IK has failed, next trying: ",\
                #    self.left_limb_state.driveSpeedAdjustment)
                return 1,0 # 1 means the IK has failed partially and we should do this again
        else:
            target_config = self.robot_model.getConfig()[10:16]
            self.left_limb_state.driveTransform = target_transform    
            if self.left_limb_state.driveSpeedAdjustment < 1:
                self.left_limb_state.driveSpeedAdjustment = self.left_limb_state.driveSpeedAdjustment + 0.1
    
        self.robot_model.setConfig(initialConfig)

        return 2,target_config #2 means success..

    def _right_limb_cartesian_drive(self,current_transform):
        v = self.right_limb_state.cartesianDriveV
        w = self.right_limb_state.cartesianDriveW
        amount = self.dt * self.right_limb_state.driveSpeedAdjustment
        #print("Before:",self.right_limb_state.driveTransform)
        #print(v,amount,vectorops.mul(v,amount))
        target_transform = (so3.mul(so3.from_moment(vectorops.mul(w,amount)),\
            self.right_limb_state.driveTransform[0]),vectorops.add(\
            self.right_limb_state.driveTransform[1],vectorops.mul(v,amount)))
        #print("After:",self.right_limb_state.driveTransform)
        #joint position limits from the joint speed limit
        joint_upper_limits = vectorops.add(self.right_limb_state.sensedq,vectorops.mul(\
            TRINAConfig.limb_velocity_limits,self.dt))
        joint_lower_limits = vectorops.add(self.right_limb_state.sensedq,vectorops.mul(\
            TRINAConfig.limb_velocity_limits,-self.dt))
        if self.left_limb_state.cartesianMode == 0:
            goal = ik.objective(self.right_EE_link,R=target_transform[0],\
                t = vectorops.sub(target_transform[1],so3.apply(target_transform[0],self.right_limb_state.toolCenter)))
        elif self.right_limb_state.cartesianMode == 1:
            goal = ik.objective(self.right_EE_link,local = [0,0,0], \
                world = vectorops.sub(target_transform[1],so3.apply(target_transform[0],self.right_limb_state.toolCenter)))

        initialConfig = self.robot_model.getConfig()
        res = ik.solve_nearby(goal,maxDeviation=0.5,activeDofs = self.right_active_Dofs,tol=0.000001)
        failFlag = False
        if res:
            if self._arm_is_in_limit(self.robot_model.getConfig()[10:16],joint_upper_limits,joint_lower_limits):
                pass
            else:
                failFlag = True
        else:
            failFlag = True
            #print("motion.controlLoop():IK solution not found")
        if failFlag:
            self.right_limb_state.driveSpeedAdjustment = self.right_limb_state.driveSpeedAdjustment - 0.1
            if self.right_limb_state.driveSpeedAdjustment < 0.001:
                self.right_limb_state.cartesianDrive = False
                print("motion.controlLoop():CartesianDrive IK has failed completely,exited..")
                return 0,0 # 0 means the IK has failed completely
            else:
                #print("motion.controlLoop():CartesianDrive IK has failed, next trying: ",\
                #    self.right_limb_state.driveSpeedAdjustment)
                return 1,0 # 1 means the IK has failed partially and we should do this again
        else:
            target_config = self.robot_model.getConfig()[27:33]
            self.right_limb_state.driveTransform = target_transform    
            if self.right_limb_state.driveSpeedAdjustment < 1:
                self.right_limb_state.driveSpeedAdjustment = self.right_limb_state.driveSpeedAdjustment + 0.1
    
        self.robot_model.setConfig(initialConfig)

        return 2,target_config #2 means success..

if __name__=="__main__":

    robot = Motion(mode = 'Kinematic')
    robot.startup()
    print('Robot start() called')
    leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
    leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
    rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
    rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)

    #move to untucked position
    robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
    robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
    startTime = time.time()
    world = robot.getWorld()
    vis.add("world",world)
    vis.show()
    while (time.time()-startTime < 5):
        vis.lock()
        robot.setBaseVelocity([0.5,0.1])
        vis.unlock()
        time.sleep(0.02)

        print(time.time()-startTime)
    robot.setBaseVelocity([0,0])
    robot.setGripperPosition([1,1,1,0])
    startTime = time.time()
    while (time.time()-startTime < 5):
        vis.lock()
        #robot.setBaseVelocity([0.5,0.1])
        vis.unlock()
        time.sleep(0.02)
    ##cartesian drive...
    #startTime = time.time()
    #robot.setLeftEEVelocity(v = [0.0,0,0], w = [0,0,0.2],tool = [0.1,0,0])
    #while (time.time()-startTime < 5):
    #    vis.lock()
        # #robot.setBaseVelocity([0.5,0.1])
        # vis.unlock()
        # time.sleep(0.02)
        # if robot.cartesian_drive_fail():
        #     break
        # print(time.time()-startTime)

    vis.kill()
    robot.shutdown()
