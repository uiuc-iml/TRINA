import os
import signal
import sys
import time
import math
from threading import Thread, Lock, RLock
import threading
import numpy as np
import TRINAConfig #network configs and other configs
from motionStates import * #state structures
from copy import deepcopy,copy
from klampt.math import vectorops,so3,se3
# from klampt import vis
from klampt.model import ik, collide
import numpy as np
import scipy as sp
from klampt import WorldModel,vis
import os

import sys
sys.path.append("..")
import trina_logging
import logging
from datetime import datetime

filename = "errorLogs/logFile_" + datetime.now().strftime('%d_%m_%Y') + ".log"
logger = trina_logging.get_logger(__name__,logging.INFO, filename)

class Motion:

    def __init__(self,mode = 'Kinematic', components = ['left_limb','right_limb'], debug_logging = False, codename = 'seed'):
        """
        This class provides a low-level controller to the TRINA robot.

        Parameters
        ------------
        mode: The 'Kinematic' mode starts a kinematic simlation of the robot. The 'Physical' mode interfaces with
            the robotic hardware directly.
        model_path: The TRINA robot model path.
        components: In the physical mode, we would like to have the option of starting only a subset of the components.
            It is a list of component names, including: left_limb, right_limb, base, torso (including the support legs.),
            left_gripper, right_gripper.
        """
        self.codename = codename
        self.mode = mode
        self.model_path = "data/TRINA_world_" + self.codename + ".xml"
        self.computation_model_path = "data/TRINA_world_" + self.codename + ".xml"
        self.debug_logging = debug_logging
        if(self.debug_logging):
            self.logging_filename = time.time()
            self.logging_file = 'teleoperation_log/log_file_' + time.strftime('%Y')+'_'+time.strftime('%m')+'_'+time.strftime('%d')+'_'+time.strftime('%H')+'_'+time.strftime('%M')+'_'+time.strftime('%S')+'.csv'

            if(os.path.exists(self.logging_file)):
                pass
            else:
                with open(self.logging_file,'w') as f:
                    f.write('arm|ik_time|collision_check_time|current_position|target_position|iterations|condition_number\r\n')
                    f.close()

            self.log_file = open(self.logging_file,'a')

        #Klampt world and robot and  used for computation
        self.world = WorldModel()
        res = self.world.readFile(self.computation_model_path)
        if not res:
            logger.error('unable to load model')
            raise RuntimeError("unable to load model")

        #Initialize collision detection
        self.collider = collide.WorldCollider(self.world)
        self.robot_model = self.world.robot(0)
        #UR5 arms need correct gravity vector
        self.currentGravityVector = [0,0,-9.81]

        #Enable some components of the robot

        self.base_enabled = False
        self.torso_enabled = False
        self.left_gripper_enabled = False
        self.right_gripper_enabled = False
        self.head_enabled = False

        left_limb_controller = None
        right_limb_controller = None

        #Initialize components
        if self.mode == "Kinematic":
            from kinematicController import KinematicController

            self.base_enabled = True
            print("initiating Kinematic controller")
            self.simulated_robot = KinematicController(self.model_path,codename = self.codename)
            # TODO: KinematicLimbController implementation
            left_limb_controller = KinematicLimbController(self.simulated_robot.getLeftLimbConfig,
                                                            self.simulated_robot.getLeftLimbVelocity,
                                                            self.simulated_robot.setLeftLimbConfig,
                                                            lambda x: None,
                                                            #self.simulated_robot.setLeftLimbVelocity,
                                                            self.simulated_robot.newState)
            right_limb_controller = KinematicLimbController(self.simulated_robot.getRightLimbConfig,
                                                            self.simulated_robot.getRightLimbVelocity,
                                                            self.simulated_robot.setRightLimbConfig,
                                                            lambda x: None,
                                                            #self.simulated_robot.setRightLimbVelocity,
                                                            self.simulated_robot.newState)
            print("initiated Kinematic controller")

        elif self.mode == "Physical":
            if 'left_limb' in components or 'right_limb' in components:
                from limbController import LimbController
            for component in components:
                if component == 'left_limb':
                    left_limb_controller = LimbController(TRINAConfig.left_limb_address,gripper=TRINAConfig.left_Robotiq,type = TRINAConfig.left_Robotiq_type,\
                        gravity = TRINAConfig.get_left_gravity_vector_upright(self.codename),payload = TRINAConfig.left_limb_payload,cog = TRINAConfig.left_limb_cog)

                    logger.debug('left limb enabled')
                elif component == 'right_limb':
                    right_limb_controller = LimbController(TRINAConfig.right_limb_address,gripper=TRINAConfig.right_Robotiq,type = TRINAConfig.right_Robotiq_type,\
                        gravity = TRINAConfig.get_right_gravity_vector_upright(self.codename),payload = TRINAConfig.right_limb_payload,cog = TRINAConfig.right_limb_cog)

                    logger.debug('right limb enabled')
                elif component == 'base':
                    from baseController import BaseController
                    self.base = BaseController()
                    self.base_enabled = True
                    logger.debug('base enabled')
                elif component == 'torso':
                    from torsoController import TorsoController
                    self.torso = TorsoController()
                    self.torso_enabled = True
                    logger.debug('torso enabled')
                elif component == 'left_gripper':
                    from gripperController import GripperController
                    self.left_gripper = GripperController()
                    self.left_gripper_enabled = True
                    logger.debug('left gripper enabled')
                elif component == 'right_gripper':
                    from gripperController import GripperController
                    self.right_gripper = GripperController()
                    self.right_gripper_enabled = True
                    logger.debug('right gripper enabled')
                elif component == "head":
                    from headController import HeadController
                    self.head = HeadController()
                    self.head_enabled = True
                    logger.debug('head enabled')
                else:
                    logger.error('Motion: wrong component name specified')
                    raise RuntimeError('Motion: wrong component name specified')
        else:
            logger.error('Wrong Mode specified')
            raise RuntimeError('Wrong Mode specified')

        #End-effector links and active dofs used for arm cartesian control and IK
        self.left_EE_link = self.robot_model.link(TRINAConfig.get_left_tool_link_N(self.codename))
        self.left_active_Dofs = TRINAConfig.get_left_active_Dofs(self.codename)
        self.right_EE_link = self.robot_model.link(TRINAConfig.get_right_tool_link_N(self.codename))
        self.right_active_Dofs = TRINAConfig.get_right_active_Dofs(self.codename)

        self.left_limb = Limb("left", self.left_EE_link, self.left_active_Dofs,
                                lambda: TRINAConfig.get_wrench_R_left(self.codename), left_limb_controller)
        self.right_limb = Limb("right", self.right_EE_link, self.right_active_Dofs,
                                lambda: TRINAConfig.get_wrench_R_right(self.codename), right_limb_controller)

        self.base_state = BaseState()
        self.torso_state = TorsoState()
        self.left_gripper_state = GripperState()
        self.head_state = HeadState()


        self.startTime = time.time()
        #time since startup
        self.t = 0
        self.startUp = False
        #Control loop rate
        self.dt = 0.002
        #automatic mode for future
        self.automatic_mode = False
        self.stop_motion_flag = False
        self.stop_motion_sent = False
        self.shut_down_flag = False
        self.cartesian_drive_failure = False
        self._controlLoopLock = RLock()
        #signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl-c)

    def sigint_handler(self, signum, frame):
        """ Catch Ctrl+C tp shutdown the robot

        """
        assert(signum == signal.SIGINT)
        logger.warning('SIGINT caught...shutting down the api!')
        print("SIGINT caught...shutting down the api!")
        self.shutdown()

    def time(self):
        """Time since the controller has started

        return:
        ---------------
        float: time since the controller has started in secs

        """
        return self.t

    def startup(self):
        """ Starts up all the individual components and the main control thread.

        Each component is started sequentially. After starting, all components stay where they are and
        start updating their states immediately.
        """

        if not self.startUp:
            if self.mode == "Kinematic":
                self.simulated_robot.start()
            elif self.mode == "Physical":
                if self.torso_enabled:
                    self.torso.start()
                    logger.info('Motion: torso started')
                    print("Motoin: torso started")
                if self.base_enabled:
                    self.base.start()
                    logger.info('Motion: base started')
                    print("Motion: base started")
                if self.left_limb.enabled or self.right_limb.enabled:
                    if self.torso_enabled:
                        #TODO read torso position
                        #tilt_angle =
                        pass
                    else:
                        tilt_angle = 0.0
                    #R_tilt = so3.from_axis_angle(([0,1,0],tilt_angle))
                    #R_local_global_left = so3.mul(R_tilt,TRINAConfig.R_local_global_upright_left)
                    #R_local_global_right = so3.mul(R_tilt,TRINAConfig.R_local_global_upright_right)
                    #gravity_left = so3.apply(so3.inv(R_local_global_left),[0,0,-9.81])
                    #gravity_right = so3.apply(so3.inv(R_local_global_right),[0,0,-9.81])
                    #self.left_limb.setGravity(gravity_left)
                    #self.right_limb.setGravity(gravity_right)
                if self.left_limb.enabled:
                    res = self.left_limb.start()
                    if res == False:
                        logger.error('left limb start failure.')
                        print("motion.startup(): ERROR, left limb start failure.")
                        return False
                    else:
                        logger.info('left limb started.')
                        print("motion.startup(): left limb started.")

                if self.head_enabled:
                    res = self.head.start()
                    time.sleep(0.5)
                    if res == False:
                        logger.error('head start failure.')
                        print("motion.startup(): ERROR, head start failure.")
                        return False
                    else:
                        logger.info('lhead started.')
                        print("motion.startup(): head started.")
                        self.head_state.sensedPosition = self.head.sensedPosition()

                if self.right_limb.enabled:
                    res = self.right_limb.start()
                    if res == False:
                        logger.error('right limb start failure.')
                        print("motion.startup(): ERROR, right limb start failure.")
                        return False
                    else:
                        logger.info('right limb started.')
                        print("motion.startup(): right limb started.")

                if self.left_gripper_enabled:
                    self.left_gripper.start()
                    print('left gripper started')
                    logger.info('left gripper started')
                if self.right_gripper_enabled:
                    self.right_gripper.start()
                    logger.info('right gripper started')


            controlThread = threading.Thread(target = self._controlLoop)
            self.shut_down_flag = False
            self.startUp = True
            controlThread.start()
            logger.info('robot started.')
            print("motion.startup():robot started")


        else:
            ##warning
            logger.warning('Already started.')
            print("motion.startup():Already started")
        return self.startUp

    def _controlLoop(self):
        """main control thread, synchronizing all components
        in each loop,states are updated and new commands are issued
        """

        counter = 0
        self.robot_start_time = time.time()
        logger.info('controlLoop started.')
        print("motion.controlLoop(): controlLoop started.")
        while not self.shut_down_flag:
            loopStartTime = time.time()
            self.t = time.time() - self.startTime
            ###lock the thread
            self._controlLoopLock.acquire()
            if self.mode == "Physical":
                if self.stop_motion_flag:
                    if not self.stop_motion_sent: #send only once to avoid drifting... currently this not used
                        if self.left_limb.enabled:
                            self.left_limb.stopMotion()
                        if self.right_limb.enabled:
                            self.right_limb.stopMotion()

                        if self.torso_enabled:
                            self.torso.stopMotion()
                        if self.base_enabled:
                            self.base.stopMotion()
                        if self.left_gripper_enabled:
                            self.left_gripper.stopMotion()
                        if self.right_gripper_enabled:
                            self.right_gripper.stopMotion()
                        if self.head_enabled:
                            pass
                        self.stop_motion_sent = True #unused
                else:
                    #Update current state. Only read state if a new one has been posted
                    if self.base_enabled and self.base.newState():
                        self.base_state.measuredVel = self.base.getMeasuredVelocity()
                        self.base_state.measuredPos = self.base.getPosition()
                        self.base.markRead()

                    if self.torso_enabled and self.torso.newState():
                        tilt, height, _, _ = self.torso.getStates()
                        self.torso_state.measuredTilt = tilt
                        self.torso_state.measuredHeight = height
                        self.torso.markRead()

                    if self.left_limb.enabled:
                        self.left_limb.updateState()
                    if self.right_limb.enabled:
                        self.right_limb.updateState()

                    if self.left_gripper_enabled and self.left_gripper.newState():
                        self.left_gripper_state.sense_finger_set = self.left_gripper.sense_finger_set
                        self.left_gripper.mark_read()

                    if self.head_enabled and self.head.newState():
                        self.head_state.sensedPosition = self.head.sensedPosition()
                        self.head.markRead()

                    #Send Commands
                    if self.left_limb.enabled:
                        self.drive_limb(self.left_limb)

                    if self.right_limb.enabled:
                        self.drive_limb(self.right_limb)

                    #TODO:Base add set path later
                    if self.base_enabled:
                        if self.base_state.commandType == 1:
                            self.base.setCommandedVelocity(self.base_state.commandedVel)
                        elif self.base_state.commandType == 0 and not base_state.commandSent:
                            self.base_state.commandSent = True
                            self.base.setTargetPosition(self.base_state.commandedVel)
                    if self.torso_enabled:
                        if not self.torso_state.commandSent:
                           self.torso_state.commandSent = True
                           self.torso.setTargetPositions(self.torso_state.commandedHeight, self.torso_state.commandedTilt)

                    if self.left_gripper_enabled:
                        if self.left_gripper_state.commandType == 0:
                          self.left_gripper.setPose(self.left_gripper_state.command_finger_set)
                        elif self.left_gripper_state.commandType == 1:
                          self.left_gripper.setVelocity(self.left_gripper_state.command_finger_set)

                    if self.right_gripper_enabled:
                        if self.right_gripper_state.commandType == 0:
                            self.right_gripper.setPose(self.right_gripper_state.command_finger_set)
                        elif self.right_gripper_state.commandType == 1:
                            self.right_gripper.setVelocity(self.right_gripper_state.command_finger_set)

                    if self.head_enabled:
                        if self.head_state.newCommand:
                            self.head.setPosition(self.head_state.commandedPosition)
                            self.head_state.newCommand = False

                    #TODO: update this for head

                    #update internal robot model, does not use the base's position and orientation
                    #basically assumes that the world frame is the frame centered at the base local frame, on the floor.
                    robot_model_Q = TRINAConfig.get_klampt_model_q(self.codename,left_limb = self.left_limb.state.sensedq, right_limb = self.right_limb.state.sensedq)
                    #robot_model_Q = [0]*3 + [0]*7 +self.left_limb.state.sensedq+[0]*4+self.right_limb_state.sensedq+[0]*2
                    self.robot_model.setConfig(robot_model_Q)

            elif self.mode == "Kinematic":
                if self.stop_motion_flag:
                    self.simulated_robot.stopMotion()
                else:
                    if self.simulated_robot.newState():
                        self.left_limb.updateState()
                        self.right_limb.updateState()
                        self.base_state.measuredVel = self.simulated_robot.getBaseVelocity()
                        self.base_state.measuredPos = self.simulated_robot.getBaseConfig()
                        #self.left_gripper_state.sense_finger_set = selfprint("motion.controlLoop(): controlLoop started.")

                    self.drive_limb(self.left_limb)
                    self.drive_limb(self.right_limb)

                    if self.base_state.commandType == 1:
                        self.simulated_robot.setBaseVelocity(self.base_state.commandedVel)
                    elif self.base_state.commandType == 0 and not base_state.commandSent:
                        base_state.commandSent = True
                        self.base.setTargetPosition(self.base_state.commandedVel)

                    ##gripper
                    self.simulated_robot.setLeftGripperPosition(self.left_gripper_state.command_finger_set)
                    robot_model_Q = TRINAConfig.get_klampt_model_q(self.codename,left_limb = self.left_limb.state.sensedq, right_limb = self.right_limb.state.sensedq)
                    self.robot_model.setConfig(robot_model_Q)

            self._controlLoopLock.release()
            elapsedTime = time.time() - loopStartTime
            self.t = time.time() - self.startTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

        logger.info('controlThread exited.')
        self._purge_commands()
        print("motion.controlThread: exited")

    def drive_limb(self, limb):
        if limb.state.commandQueue:
            if limb.state.commandType == 0:
                tmp = time.time() - limb.state.commandQueueTime
                if tmp <= limb.state.commandedQueueDuration:
                    limb.setConfig(vectorops.add(limb.state.commandedqQueueStart,vectorops.mul(limb.state.difference,tmp/limb.state.commandedQueueDuration)))
                else:
                    limb.setConfig(vectorops.add(limb.state.commandedqQueueStart,vectorops.mul(limb.state.difference,1.0)))
                    self.setLimbPosition(limb, vectorops.add(limb.state.commandedqQueueStart,vectorops.mul(limb.state.difference,1.0)))

        #### cartesian drive mode
        elif limb.state.cartesianDrive:
            flag = 1
            while flag == 1:
                res, target_config = self._cartesian_drive(limb, limb.state.driveTransform)
                if res == 0:
                    #res = 0 means IK has failed completely, 1 means keep trying smaller steps, 2 means success
                    self.cartesian_drive_failure = True
                    #set to position mode... TODO: Do we need to limb.setConfig() as well?
                    limb.state.set_mode_position(limb.state.sensedq)
                    break
                elif res == 1:
                    flag = 1
                elif res == 2:
                    flag = 0
                    limb.setConfig(target_config)
        elif limb.state.impedanceControl:
            res,target_config = self._impedance_drive(limb)
            #res = 0 means IK has failed completely, 1 success, 2 means force overload
            if res == 0:
                # TODO: Why is this cartesian_drive_failure?
                self.cartesian_drive_failure = True
                limb.state.set_mode_position(limb.state.sensedq)
            elif res == 1:
                limb.setConfig(target_config)
            elif res == 2:
                # TODO: I'm concerned.
                self.setLimbPositionLinear(limb, target_config, 2)
        else:
            ### Velocity control or position control.
            if not limb.state.commandSent:
                ###setting position will clear velocity commands
                if limb.state.commandType == 0:
                    limb.setConfig(limb.state.commandedq)
                elif limb.state.commandType == 1:
                    limb.setVelocity(limb.state.commandeddq)
                limb.state.commandSent = True

    #TODO: finish setting the entire robot
    def setPosition(self,q):
        """set the position of the entire robot

        Parameter:
        ---------------
        q: a merged list of joint positions, in the order of torso,base,left limb, right limb, left gripper...
        """
        #assert len(q) == 12, "motion.setPosition(): Wrong number of dimensions of config sent"
        #self.setLeftLimbPosition(q[0:6])
        #self.setRightLimbPosition(q[6:12])
        pass
        return

    # TODO: Refactor incomplete
    def setLimbPosition(self, limb, q):
        """Set the left limb joint positions, and the limb moves as fast as possible

        This will clear the motion queue.

        Parameter:
        --------------
        limb: Limb to set.
        q: a list of 6 doubles. The desired joint positions.
        """
        if limb.name == "left":
            self.setLeftLimbPosition(q)
        else:
            self.setRightLimbPosition(q)

    def setLeftLimbPosition(self,q):
        """Set the left limb joint positions, and the limb moves as fast as possible

        This will clear the motion queue.

        Parameter:
        --------------
        q: a list of 6 doubles. The desired joint positions.
        """
        logger.debug('number of joint positions sent : %d', len(q))
        assert len(q) == 6 #, "motion.setLeftLimbPosition(): Wrong number of joint positions sent"('controlThread exited.')
        if self.left_limb.enabled:
            self._controlLoopLock.acquire()
            # TODO ????? (Jing-Chen)
            self._check_collision_linear_adaptive(self.robot_model,self._get_klampt_q(left_limb = self.left_limb.state.sensedq),self._get_klampt_q(left_limb = q))
            self.left_limb.state.set_mode_position(q)
            self._controlLoopLock.release()
        else:
            logger.warning('Left limb not enabled')
            print("motion.setLeftLimbPosition():Left limb not enabled")
        return

    def setRightLimbPosition(self,q):
        """Set the right limb joint positions, and the limb moves as fast as possible

        This will clear the motion queue.

        Parameter:
        --------------
        q: a list of 6 doubles. The desired joint positions.
        """
        logger.debug('number of joint positions sent : %d', len(q))
        assert len(q) == 6, "motion.setLeftLimbPosition(): Wrong number of joint positions sent"
        if self.right_limb.enabled:
            self._controlLoopLock.acquire()
            # TODO ????? (Jing-Chen)
            self._check_collision_linear_adaptive(self.robot_model,self._get_klampt_q(right_limb = self.right_limb.state.sensedq),self._get_klampt_q(right_limb = q))
            self.right_limb.state.set_mode_position(q)
            self._controlLoopLock.release()
        else:
            logger.warning('Right limb not enabled')
            print("motion.setRightLimbPosition():Right limb not enabled")
        return

    # TODO: Refactor incomplete
    def setLimbPositionLinear(self, limb, q, duration):
        """Set Left limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameter:
        --------------
        limb: Limb to set.
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
        if limb.name == "left":
            self.setLeftLimbPositionLinear(q, duration)
        else:
            self.setRightLimbPositionLinear(q, duration)


    def setLeftLimbPositionLinear(self,q,duration):
        """Set Left limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameters:
        ----------------
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
        logger.debug('number of joint positions sent : %d and duration is %d', len(q), duration)
        assert len(q) == 6, "motion.setLeftLimbPositionLinear(): Wrong number of joint positions sent"
        assert duration > 0, "motion.setLeftLimbPositionLinear(): Duration needs to be a positive number"
        #TODO:add velocity check. Maybe not be able to complete the motion within the duration"
        #TODO:Also collision checks
        if self.left_limb.enabled:
            self._controlLoopLock.acquire()
            # NOTE: Why are we running collision checks and tossing the results? WHY? (Jing-Chen)
            self._check_collision_linear_adaptive(self.robot_model,self._get_klampt_q(left_limb = self.left_limb.state.sensedq),self._get_klampt_q(left_limb = q))
            #planningTime = 0.0 + TRINAConfig.ur5e_control_rate
            #positionQueue = []
            #currentq = self.left_limb.state.sensedq
            #difference = vectorops.sub(q,currentq)
            #while planningTime < duration:
            #    positionQueue.append(vectorops.add(currentq,vectorops.mul(difference,planningTime/duration)))
            #    planningTime = planningTime + self.dt #TRINAConfig.ur5e_control_rate
            #positionQueue.append(q)
            difference = vectorops.sub(q,self.left_limb.state.sensedq)
            start = self.left_limb.state.sensedq
            self.left_limb.state.set_mode_commandqueue(difference, start, duration)
            self._controlLoopLock.release()
        else:
            logger.warning('Left limb not enabled')
            print("motion.setLeftLimbPosition():Left limb not enabled")
        print

    def setRightLimbPositionLinear(self,q,duration):
        """Set right limb to moves to a configuration in a certain amount of time at constant speed

        Set a motion queue, this will clear the setPosition() commands

        Parameters:
        ----------------
        q: a list of 6 doubles. The desired joint positions.
        duration: double. The desired duration.
        """
        logger.debug('number of joint positions sent : %d and duration is %d', len(q), duration)
        assert len(q) == 6, "motion.setRightLimbPositionLinear(): Wrong number of joint positions sent"
        assert duration > 0, "motion.setRightLimbPositionLinear(): Duration needs to be a positive number"
        #TODO:add velocity check. Maybe not be able to complete the motion within the duration"
        #Also collision checks
        if self.right_limb.enabled:
            self._controlLoopLock.acquire()
            # NOTE: Why are we running collision checks and tossing the results? WHY? (Jing-Chen)
            self._check_collision_linear_adaptive(self.robot_model,self._get_klampt_q(right_limb = self.right_limb.state.sensedq),self._get_klampt_q(right_limb = q))
            difference = vectorops.sub(q,self.right_limb.state.sensedq)
            start = self.right_limb.state.sensedq
            self.right_limb.state.set_mode_commandqueue(difference, start, duration)
            self._controlLoopLock.release()
        else:
            logger.warning('Right limb not enabled')
            print("motion.setRightLimbPosition():Right limb not enabled")
        return




    def sensedLeftLimbPosition(self):
        """The current joint positions of the left limb

        Return:
        --------------
        A list of 6 doubles. The limb configuration.
        """
        if self.left_limb.enabled:
            return copy(self.left_limb.state.sensedq)
        else:
            logger.warning('left limb not enabled')
            print("motion().sensedLeftLimbPosition: left limb not enabled")
            return [0]*6

    def sensedRightLimbPosition(self):
        """The current joint positions of the right limb

        Return:
        --------------
        A list of 6 doubles. The limb configuration.
        """
        if self.right_limb.enabled:
            return copy(self.right_limb.state.sensedq)
        else:
            logger.warning('Right limb not enabled')
            print("motion().sensedRightLimbPosition: right limb not enabled")
            return [0]*6

    def setVelocity(self,qdot):
        """set the velocity of the entire robot, under development rn

        """
        #assert len(qdot) == 12, "motion.setPosition(): Wrong number of dimensions of config sent"
        #self.setLeftLimbVelocity(qdot[0:6])
        #self.setRightLimbVelcity(qdot[6:12])
        pass
        return

    def setLeftLimbVelocity(self,qdot):
        """Set the left limb joint velocities

        Parameter:
        ----------------
        qdot: a list of 6 doubles. Joint velocities
        """
        if self.left_limb.enabled:
            logger.debug('number of joint velocities sent : %d', len(qdot))
            assert len(qdot) == 6, "motion.setLeftLimbVelocity()): Wrong number of joint velocities sent"
            self._controlLoopLock.acquire()
            self.left_limb.state.set_mode_velocity(qdot)
            self._controlLoopLock.release()
        else:
            logger.warning('Left limb not enabled')
            print("Left limb not enabled")

        return

    def setRightLimbVelocity(self,qdot):
        """Set the right limb joint velocities

        Parameter:
        ----------------
        qdot: a list of 6 doubles. Joint velocities
        """
        if self.right_limb.enabled:
            logger.debug('number of joint velocities sent : %d', len(qdot))
            assert len(qdot) == 6, "motion.setRightLimbVelocity()): Wrong number of joint velocities sent"
            self._controlLoopLock.acquire()
            self.right_limb.state.set_mode_velocity(qdot)
            self._controlLoopLock.release()
        else:
            logger.warning('Right limb not enabled')
            print("Right limb not enabled.")
        return

    def setEEInertialTransform(self,limb,Ttarget,duration):
        """Set the trasform of the arm w.r.t. the base frame, movement complete in a certain amount of time

        This current version assmumes that the torso is at zero position.
        #TODO: implement version with torso not at zero.

        Parameter:
        ---------------
        limb: Limb to set.
        Ttarget: A klampt rigid transform (R,t). R is a column major form of a rotation matrix. t is a 3-element list
        duration: double. The duration of the movement

        Return:
        ---------------
        Status string (empty on success)
        """
        start_time = time.time()
        if limb.enabled:
            self._controlLoopLock.acquire()

            initial = self.robot_model.getConfig()

            goal = ik.objective(limb.EE_link,R=Ttarget[0],t = Ttarget[1])
            if ik.solve(goal,activeDofs = limb.active_dofs):
            #if ik.solve_global(goal,activeDofs = limb.active_dofs):
            # if ik.solve_nearby(goal,maxDeviation=3,activeDofs = limb.active_dofs):
            #if result:
                target_config = self.robot_model.getConfig()
                logger.info('IK solve successful')
                print(f"motion.setEEInertialTransform({limb.name}):IK solve successful")
            else:
                self.robot_model.setConfig(initial)
                self._controlLoopLock.release()
                logger.warning('IK solve failure: no IK solution found')
                status = f'motion.setEEInertialtransform({limb.name}):IK solve failure: no IK solution found'
                print(status)
                return status

            ik_solve_time = time.time() -start_time
            # print("Solving IK takes", time.time() -start_time,' and {} iterations'.format(iterations))
            start_time_2 = time.time()
            res = self._check_collision_linear_adaptive(self.robot_model,initial,target_config)
            col_check_time = time.time()-start_time_2
            # print("collision checking takes", time.time() - start_time_2)
            if res:
                self._controlLoopLock.release()
                logger.warning('Self-collision midway')
                status = f'motion.setEEInertialTransform({limb.name}): Self-collision midway'
                print(status)
                return status
            else:
                logger.info('No collision')
                print(f"motion.setEEInertialTransform({limb.name}):No collision")

            self.robot_model.setConfig(initial)
            self._controlLoopLock.release()
            start_time = time.time()
            # array convert for index-by-list.
            self.setLimbPositionLinear(limb, np.array(target_config)[self.left_active_Dofs],duration)
            # print("setting linear position takes", time.time() - start_time)
        else:
            status = f'{limb.name} limb not enabled'
            logger.warning(status)
            print(status)
            return status
        return ''

    def setLeftEEInertialTransform(self,Ttarget,duration):
        return self.setEEInertialTransform(self.left_limb, Ttarget, duration)

    def setRightEEInertialTransform(self,Ttarget,duration):
        return self.setEEInertialTransform(self.right_limb, Ttarget, duration)

    def setEEVelocity(self, limb, v, tool = [0,0,0]):
        """Set the end-effect cartesian velocity, in the base frame.

        Implemented using position control and IK. Will keep moving until infeasible.
        TODO: implement collision detection

        Parameter:
        --------------
        limb: Limb to set.
        v: A list of 6 doubled. v[0:3] is the desired cartesian position velocities and v[3:6] is the desired rotational velocity
        tool: Tool center. Used for rotations.

        Return:
        --------------
        Status string

        """
        if limb.enabled:
            self._controlLoopLock.acquire()
            if len(v) == 3:
                limb.state.cartesianDriveV = deepcopy(v)
                limb.state.cartesianMode = 1

            elif len(v) == 6:
                limb.state.cartesianDriveV = deepcopy(v[0:3])
                limb.state.cartesianDriveW = deepcopy(v[3:6])
                limb.state.cartesianMode = 0
            else:
                limb.state.set_mode_position(limb.state.sensedq)
                #error
                logger.error('wrong input')
                status = f"motion.setEEVelocity({limb.name}): wrong input"
                print(status)
                self._controlLoopLock.release()
                return status

            if not limb.state.cartesianDrive:
                limb.state.set_mode_position(limb.state.sensedq)
                self.cartesian_drive_failure = False

                ##cartesian velocity drive
                limb.state.cartesianDrive = True
                limb.state.driveTransform = (R,vectorops.add(so3.apply(R,tool),t))

            (R,t) = limb.sensedEETransform()
            limb.state.startTransform = (R,vectorops.add(so3.apply(R,tool),t))
            limb.state.driveSpeedAdjustment = 1.0
            limb.state.toolCenter = deepcopy(tool)
            self._controlLoopLock.release()
        else:
            status = f"{limb.name} limb not enabled."
            logger.warning(status)
            print(status)
            return status
        return ''

    def setLeftEEVelocity(self,v, tool = [0,0,0]):
        self.setEEVelocity(self.left_limb, v, tool)

    def setRightEEVelocity(self,v, tool = [0,0,0]):
        self.setEEVelocity(self.right_limb, v, tool)

    def setEETransformImpedance(self,limb,Tg,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center = [0,0,0]):
        """Set the target transform of the EE in the global frame. The EE will follow a linear trajectory in the cartesian space to the target transform.
        The EE will behave like a spring-mass-damper system attached to the target transform. The user will need to supply the elasticity matrix, the damping matrix,
        and the inertia matrix

        Parameters:
        -------------
        limb: Limb to set.
        Tg: target transform of the EE, in Klampt format
        K: a 6x6 numpy 2D array. The elasticity matrix, this should be a diagonal matrix. The ordering is that the first 3 diagonal entries are for translations.
        B: a 6x6 numpy 2D array. The damping matrix.
        M: a 6x6 numpy 2D array. The inertia matrix.
        x_dot_g: list of 6 elements. The optional desired EE velocity
        deadband: list of 6 elements. This is the range for ignoring the wrench readings (kind of like "deadband")
        tool_center: Tool center (offset from the end effector in local frame of last link)

        Return:
        -------------
        None
        """
        if not limb.enabled:
            status = f'setEETransformImpedance({limb.name}): limb not enabled.'
            print(status)
            logger.warning(status)
            return 0
#        if self.mode == "Kinematic":
#            print("SetLeftEETransform():Impedance control not available for Kinematic mode.")
#            logger.warning('SetLeftEETransformImpedance():Impedance control not available for Kinematic mode.')
#            return 0

        if np.shape(K) != (6,6) or np.shape(M) != (6,6):
            status = f'setEETransformImpedance({limb.name}): Wrong shape for inputs.'
            print(status)
            logger.warning(status)
            return 0

        if np.all(K<0) or np.all(M<0):
            status = f'setEETransformImpedance({limb.name}): K,M need to be nonnegative.'
            print(status)
            logger.warning(status)
            return 0

        if type(x_dot_g) is not list:
            status = f'setEETransformImpedance({limb.name}): x_dot_g needs to be a list.'
            print(status)
            logger.warning(status)
            return 0

        self._controlLoopLock.acquire()

        formulation = 2
        #if already in impedance control, then do not reset x_mass and x_dot_mass 
        if (not limb.state.impedanceControl) or vectorops.norm(vectorops.sub(limb.state.toolCenter,tool_center)):
            limb.state.set_mode_reset()
            if formulation == 2:
                limb.state.T_mass = limb.sensedEETransform(tool_center = tool_center)
            elif formulation == 1:
                T = limb.sensedEETransform(tool_center = [0, 0, 0])
                limb.state.x_mass = T[1] + so3.moment(T[0])
            (v,w) = limb.sensedEEVelocity(tool_center)
            limb.state.x_dot_mass = v+w
            limb.state.toolCenter = copy(tool_center)
            limb.state.prev_wrench = np.array([0]*6)

        if formulation == 2:
            limb.state.T_g = copy(Tg)
        elif formulation == 1:
            limb.state.x_g = Tg[1] + so3.moment(Tg[0])
        
        limb.state.impedanceControl = True
        
        limb.state.x_dot_g = copy(x_dot_g)
        limb.state.K = np.copy(K)
        limb.state.counter = 1
        limb.state.deadband = copy(deadband)
        if np.any(np.isnan(B)):
            limb.state.B = np.sqrt(4.0*np.dot(M,K))
        else:
            limb.state.B = np.copy(B)
        Minv = np.linalg.inv(M)
        limb.state.Minv = Minv
        tmp = np.vstack( (np.hstack((np.zeros((6,6)), np.eye(6))), 
            np.hstack((-Minv @ K, -Minv @ B))) )
        limb.state.A = np.eye(12) - self.dt*tmp
        # limb.state.LU = sp.linalg.lu_factor(limb.state.A)
        self._controlLoopLock.release()
        return 0

    def setLeftEETransformImpedance(self,Tg,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center = [0,0,0]):
        return self.setEETransformImpedance(self.left_limb, Tg, K, M, B, x_dot_g, deadband, tool_center)

    def setRightEETransformImpedance(self,Tg,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center = [0,0,0]):
        return self.setEETransformImpedance(self.right_limb, Tg, K, M, B, x_dot_g, deadband, tool_center)

    def setLimbPositionImpedance(self,limb,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center=[0]*3):
        """Set the target position of the limb. The EE will follow a linear trajectory in the cartesian space to the target transform.
        The EE will behave like a spring-mass-damper system attached to the target transform. The user will need to supply the elasticity matrix, the damping matrix,
        and the inertia matrix

        Parameters:
        -------------
        limb: Limb to set.
        q: target positin of the limb
        K: a 6x6 numpy 2D array. The elasticity matrix, this should be a diagonal matrix. The ordering is that the first 3 diagonal entries are for translations.
        B: a 6x6 numpy 2D array. The damping matrix.
        M: a 6x6 numpy 2D array. The inertia matrix.
        x_dot_g: list of 6 elements. The optional desired EE velocity
        deadband: list of 6 elements. This is the range for ignoring the wrench readings (kind of like "deadband")
        tool_center: Tool center (offset from the end effector in local frame of last link)

        Return:
        -------------
        None
        """
        initialConfig = self.robot_model.getConfig()
        currentConfig = np.array(initialConfig)
        currentConfig[limb.active_dofs] = q
        self.robot_model.setConfig(currentConfig)
        EETransform = self.left_EE_link.getTransform()
        self.robot_model.setConfig(initialConfig)
        self.setEETransformImpedance(limb,EETransform,K,M,B,x_dot_g,deadband,tool_center)
        return 0

    def setLeftLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center=[0]*3):
        return self.setLimbPositionImpedance(self.left_limb, q, K, M, B, x_dot_g, deadband, tool_center)

    def setRightLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center=[0]*3):
        return self.setLimbPositionImpedance(self.right_limb, q, K, M, B, x_dot_g, deadband, tool_center)

    def sensedLeftEETransform(self,tool_center=[0,0,0]):
        """Return the transform of the tool position w.r.t. the base frame
        Parameters:
        -------------
        tool_center: Tool center (or none to use state tool center)

        Return:
        -------------
        (R,t)
        """
        if self.left_limb.enabled:
            # Destroying internal consistency.
            return self.left_limb.sensedEETransform(tool_center=tool_center)
        else:
            logger.warning('Left limb not enabled.')
            print("Left limb not enabled.")
            return

    def sensedLeftEEVelocity(self,local_pt = [0,0,0]):
        """Return the EE translational and rotational velocity  w.r.t. the base Frame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
        if self.left_limb.enabled:
            return self.left_limb.sensedEEVelocity(local_pt)
        else:
            return "NA"

    def sensedRightEETransform(self,tool_center=[0,0,0]):
        """Return the transform w.r.t. the base frame

        Return:
        -------------
        (R,t)
        """
        if self.right_limb.enabled:
            return self.right_limb.sensedEETransform(tool_center=tool_center)
        else:
            logger.warning('Right limb not enabled.')
            print("Right limb not enabled.")
            return

    def sensedRightEEVelocity(self,local_pt = [0,0,0]):
        """Return the EE translational and rotational velocity  w.r.t. the base Frame

        Parameter:
        ----------------
        local_pt: the local point in the EE local frame.

        Return:
        ----------------
        (v,w), a tuple of 2 velocity vectors

        """
        if self.right_limb.enabled:
            return self.right_limb.sensedEEVelocity(local_pt)
        else:
            return "NA"

    def sensedLeftLimbVelocity(self):
        """ Return the current limb joint velocities

        Return:
        ---------------
        A list of 6 doubles. The joint velocities.
        """
        if self.left_limb.enabled:
            return copy(self.left_limb.state.senseddq)
        else:
            logger.warning('Left limb not enabled.')
            print("Left limb not enabled.")
            return

    def sensedRightLimbVelocity(self):
        """ Return the current limb joint velocities

        Return:
        ---------------
        A list of 6 doubles. The joint velocities.
        """
        if self.right_limb.enabled:
            return copy(self.right_limb.state.senseddq)
        else:
            logger.warning('Right limb not enabled.')
            print('Right limb not enabled.')
            return

    def setBaseTargetPosition(self, q, vel):
        """Set the local target position of the base.

        The base constructs a path to go to the desired position, following the desired speed along the path
        Parameter:
        ---------------
        q: a list of 3 doubles. The desired x,y position and rotation.
        Vel: double. Desired speed along the path.
        """
        if self.base_enabled:
            logger.debug('dimensions : %d', len(q))
            assert len(q) == 3, "motion.setBaseTargetPosition(): wrong dimensions"
            self._controlLoopLock.acquire()
            self.base_state.commandType = 0
            self.base_state.commandedTargetPosition = deepcopy(q)
            self.base_state.pathFollowingVel = vel
            self.base_state.commandSent = False
            self._controlLoopLock.release()
        else:
            logger.warning('Base not enabled.')
            print('Base not enabled.')

    def setBaseVelocity(self, q):
        """Set the velocity of the base relative to the local base frame

        Parameter:
        ---------------
        q: a list of 2 doubles. The linear and rotational velocites.
        """
        if self.base_enabled:
            logger.debug('dimensions : %d', len(q))
            assert len(q) == 2 ,"motion.setBaseVelocity(): wrong dimensions"
            self._controlLoopLock.acquire()
            self.base_state.commandType = 1
            self.base_state.commandedVel = deepcopy(q)
            self.base_state.commandSent = False
            self._controlLoopLock.release()
        else:
            logger.warning('Base not enabled.')
            print('Base not enabled.')

    def setTorsoTargetPosition(self, q):
        """Set the torso target position.

        Moves to the target as fast as possible.

        Parameter:
        --------------
        q: a list of 2 doubles. The lift and tilt positions.
        """
        if self.torso_enabled:
            logger.debug('dimensions : %d', len(q))
            assert len(q) == 2, "motion.SetTorsoTargetPosition(): wrong dimensions"
            height, tilt = q
            self._controlLoopLock.acquire()
            self.torso_state.commandedHeight = height
            self.torso_state.commandedTilt = tilt
            self.torso_state.commandSent = False
            self._controlLoopLock.release()
        else:
            logger.warning('Torso not enabled.')
            print('Torso not enabled.')

    def setHeadPosition(self,q):
        """Set the head target position.
        Parameter:
        --------------
        q: a list of 2 doubles. The tilt and pan positions.
        """
        if self.head_enabled:
            logger.debug('dimensions : %d', len(q))
            assert len(q) == 2, "motion.SetHeadTPosition(): wrong dimensions"
            self._controlLoopLock.acquire()
            self.head_state.commandedPosition = copy(q)
            self.head_state.newCommand = True
            self._controlLoopLock.release()
        else:
            logger.warning('Head not enabled.')
            print('Head not enabled.')
            
    def sensedBaseVelocity(self):
        """Returns the current base velocity

        Return:
        -----------
        A list of 2 doubles. Linear and Rotational velocities.
        """
        if self.base_enabled:
            return copy(self.base_state.measuredVel)
        else:
            logger.warning('Base not enabled.')
            print('Base not enabled')
            return [0,0]

    def sensedBasePosition(self):
        """Returns the current base position. Zero position is the position when the base is started.

        Return:
        -------------
        A list of 3 doubles. Position and rotation.

        """
        if self.base_enabled:
            return copy(self.base_state.measuredPos)
        else:
            logger.warning('Base not enabled.')
            print('Base not enabled')
            return [0,0,0]

    def sensedTorsoPosition(self):
        """Returns the current torso position

        Return:
        -------------
        A list of 2 doubles. The positions.
        """
        if self.torso_enabled:
            return copy([self.torso_state.measuredHeight, self.torso_state.measuredTilt])
        else:
            logger.warning('Torso not enabled.')
            print('Torso not enabled.')

    def sensedHeadPosition(self):
        """Returns the current head position

        Return:
        -------------
        A list of 2 doubles. The positions.
        """
        if self.head_enabled:
            return copy(self.head_state.sensedPosition)
        else:
            logger.warning('Head not enabled.')
            print('Head not enabled.')
            return 0

    def openLeftRobotiqGripper(self):
        """ Open the parallel gripper or release the vacuum gripper. This gripper is connected to the arm.
        """
        if self.left_limb.enabled:
            self.left_limb.openGripper()
        else:
            logger.warning('Left limb not enabled.')
            print('Left limb not enabled.')
        return 0

    def closeLeftRobotiqGripper(self):
        """ close the parallel gripper or start the vacuum gripper. This gripper is connected to the arm.
        """
        if self.left_limb.enabled:
            self.left_limb.closeGripper()
        else:
            logger.warning('Left limb not enabled.')
            print('Left limb not enabled.')
        return 0
    def openRightRobotiqGripper(self):
        """ Open the parallel gripper or release the vacuum gripper. This gripper is connected to the arm.
        """
        if self.right_limb.enabled:
            self.right_limb.openGripper()
        else:
            logger.warning('Right limb not enabled.')
            print('Right limb not enabled.')
        return 0
    def closeRightRobotiqGripper(self):
        """ close the parallel gripper or start the vacuum gripper. This gripper is connected to the arm.
        """
        if self.right_limb.enabled:
            self.right_limb.closeGripper()
        else:
            logger.warning('Right limb not enabled.')
            print('Right limb not enabled.')
        return 0
    def setLeftGripperPosition(self, position):
        """Set the position of the gripper. Moves as fast as possible.

        Parameters:
        -----------------
        position: a list of 4 doubles, the angles of finger 1,2 , the angle of the thumb,
            the rotation of finger 1&2 (they rotate together)
        """
        self._controlLoopLock.acquire()
        self.left_gripper_state.commandType = 0
        self.left_gripper_state.command_finger_set = deepcopy(position)
        self._controlLoopLock.release()

    def setLeftGripperVelocity(self,velocity):
        """Set the velocity of the gripper. Moves as fast as possible.
        #TODO
        ###Under development
        """
        self.left_gripper_state.commandType = 1
        self.left_gripper_state.command_finger_set = deepcopy(velocity)

    def sensedLeftGripperPosition(self):
        """Return the current positions of the fingers.
        #TODO
        ###Under development
        """
        return copy(self.left_gripper_state.sense_finger_set)

    def getKlamptCommandedPosition(self):
        """Return the entire commanded position, in Klampt format. The base returns velocity instead
        """
        if self.left_limb.state.commandedq and self.right_limb.state.commandedq:
            return TRINAConfig.get_klampt_model_q(self.codename,left_limb = self.left_limb.state.commandedq, right_limb = self.right_limb.state.commandedq,base = self.base_state.commandedVel + [0])
        else:
            return self.getKlamptSensedPosition()

    def getKlamptSensedPosition(self):
        """Return the entire sensed Klampt position, in Klampt format.
        """
        #return TRINAConfig.get_klampt_model_q(self.codename,left_limb = self.left_limb_state.sensedq, right_limb = self.right_limb_state.sensedq,base = self.base_state.measuredPos)
        return self.robot_model.getConfig()


    def sensedLeftEEWrench(self,frame = 'global',tool_center = [0,0,0]):
        """
        Parameters:
        ------------------
        frame:  'global' or 'local'
        tool_center: Tool center

        Return:
        ------------------
        wrench: list of 6 floats, expressed either in global or local EE coordinate, gravity of the attachement compensated for

        Note:
        The attachment weight to the ft sensor can be corrected by U5 directly
        """
        if not self.left_limb.enabled:
            logger.info("sensedLeftEEWrench:left limb not enabled")
            return [0,0,0,0,0,0]
        return self.left_limb.sensedEEWrench(frame, tool_center)

    def sensedRightEEWrench(self,frame = 'global',tool_center = [0,0,0]):
        """
        Parameters:
        ------------------
        frame:  'global' or 'local'
        tool_center: Tool center
        Return:
        ------------------
        wrench: list of 6 floats, expressed either in global or local EE coordinate

        """
        if not self.right_limb.enabled:
            logger.info("sensedRightEEWrench:right limb not enabled")
            return [0,0,0,0,0,0]
        return self.right_limb.sensedEEWrench(frame, tool_center)


    def shutdown(self):
        """Shutdown the componets.

        """
        self.shut_down_flag = True
        if self.mode == "Physical":
            if self.base_enabled:
                self.base.shutdown()
            if self.torso_enabled:
                self.torso.shutdown()
            if self.left_limb.enabled:
                self.left_limb.stop()
            if self.right_limb.enabled:
                self.right_limb.stop()
            #TODO: integrate gripper code
            if self.left_gripper_enabled:
                self.left_gripper.shutDown()
            if self.head_enabled:
                self.head.shutdown()

        elif self.mode == "Kinematic":
            self.simulated_robot.shutdown()

        self._purge_commands()
        self.shut_down_flag = True
        self.startUp = False
        return 0

    def isStarted(self):
        """Return whether the robot has started

        Return:
        ------------
        bool
        """
        return self.startUp

    def isShutDown(self):
        """Return whether the robot is shutdown

        Return:
        ------------
        bool
        """
        return self.shut_down_flag

    def moving(self):
        """Returns true if the robot is currently moving.

        Return:
        ------------
        bool
        """

        if self.mode == 'Physical':
            flag = False
            if self.left_limb.enabled:
                flag = flag or self.left_limb.moving()
            if self.right_limb.enabled:
                flag = flag or self.right_limb.moving()
            if self.base_enabled:
                flag = flag or self.base.moving()
            if self.torso_enabled:
                flag = flag or self.torso.moving()
            return flag
        else:
            return self.simulated_robot.moving()

    def mode(self):
        """Returns the current mode. "Kinematic" or "Physical"

        Return:
        ------------
        string
        """
        return self.mode

    def stopMotion(self):
        """Pause the motion of the robot, starting from the next control loop.

        This is not shutdown. Just a pause.
        """
        self.stop_motion_flag = True
        self.stop_motion_sent = False
        self._purge_commands()
        return



    ### ------------------------- ###
    ###current change up to here#######
    def resumeMotion(self):
        """Unpause the robot.

        After unpausing, the robot is still stationery until some new commands is added
        """
        self.base.startMotion()
        self.startMotionFlag = False
        self.left_gripper.resume()
        return

    def mirror_arm_config(self,config):
        """given the Klampt config of the left or right arm, return the other

        Paremeters:
        ---------------
        A list of 6 doubles. Limb configuration.

        Return:
        ---------------
        A list of 6 doubles. Limb configuration.
        """
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
        """ Return the simulated robot

        Return:
        -------------
        The Klampt world of the simulated robot.
        """
        if self.mode == "Kinematic":
            return self.simulated_robot.getWorld()
        else:
            print("wrong robot mode.")

    def cartesianDriveFail(self):
        """ Return if cartesian drive has failed or not

        Return:
        ----------------
        bool
        """
        return self.cartesian_drive_failure


    def setRobotToDefualt(self):
        """ Some helper function when debugging"""
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.mirror_arm_config(leftUntuckedConfig)
        self.setLeftLimbPositionLinear(leftUntuckedConfig,1)
        self.setRightLimbPositionLinear(rightUntuckedConfig,1)

    def zeroLeftFTSensor(self):
        """
        zero the ft sensor.
        """
        if self.left_limb.enabled:
            self.left_limb.zeroFTSensor()
        else:
            logger.info("zeroLeftFTSensor:left limb not enabled")

        return 0

    def zeroRightFTSensor(self):
        """
        zero the ft sensor.
        """
        if self.right_limb.enabled:
            self.right_limb.zeroFTSensor()
        else:
            logger.info("zeroRightFTSensor:right limb not enabled")

        return 0

    ###Below are internal helper functions
    def _purge_commands(self):
        """Clear all the motion commands
        """
        self._controlLoopLock.acquire()
        if self.left_limb.enabled:
            self.left_limb.state.set_mode_reset()
        if self.right_limb.enabled:
            self.right_limb.state.set_mode_reset()
        if self.base_enabled:
            self.base_state.commandedVel = [0.0, 0.0]
            self.base_state.commandedTargetPosition = [] #[x, y, theta]
            self.base_state.commandSent = True
        if self.left_gripper_enabled:
            self.left_gripper_state.command_finger_set = [0.0, 0.0, 0.0, 0.0]
        if self.torso_enabled:
            self.torso_state.commandedHeight = 0.0
            self.torso_state.commandedTilt = 0.0
            self.torso_state.commandSent = True
            self.torso_state.leftLeg = 0
            self.torso_state.rightLeg = 0

        self._controlLoopLock.release()

    def _check_collision_linear(self,robot,q1,q2,discretization):
        """ Check collision between 2 robot configurations

        Parameters:
        -----------------
        robot: klampt robot model
        q1: a list of 6 doubles
        q2: a list of 6 doubles
        discretization: integers, the number of collision checks

        Return:
        -----------------
        bool
        """

        lin = np.linspace(0,1,discretization)
        initialConfig = robot.getConfig()
        diff = vectorops.sub(q2,q1)
        counter = 0
        for c in lin:
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
        return False

    def _check_collision_linear_adaptive(self,robot,q1,q2):
        """ Check collision between 2 robot configurations, but with adaptive number of collision checks

        Parameters:
        -----------------
        robot: klampt robot model
        q1: a list of 6 doubles
        q2: a list of 6 doubles

        Return:
        -----------------
        bool
        """
        discretization = math.ceil(vectorops.distance(q1,q2)/TRINAConfig.collision_check_interval)
        #print("N of discretization",discretization)
        lin = np.linspace(0,1,discretization + 1)
        initialConfig = robot.getConfig()
        diff = vectorops.sub(q2,q1)
        counter = 0
        iteration_counter = 0
        for c in lin:
            if iteration_counter > 0:
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
            else:
                iteration_counter = iteration_counter + 1
        robot.setConfig(initialConfig)
        return False

    def _limit_arm_position(self,config):
        """Modify the arm configuration to be within joint position limits

        Parameters:
        ---------------
        config: a list of 6 doubles

        Return:
        ---------------
        a list of 6 doubles
        """
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
        """Check if the config is within the limits

        Parameters:
        ---------------
        Lists of same length

        Return:
        ---------------
        bool
        """
        for [q,qu,ql] in zip(config,upper,lower):
            if q > qu or q < ql:
                return False

        return True

    def _cartesian_drive(self,limb,current_transform):
        ###TODO: robot_model config is no longer updated in the main loop ....

        """ Calculate the next position command for cartedian velocity drive

        Parameters:
        -------------
        limb: The limb to drive.
        current_transform: klampt rigid transform.

        Return:
        -------------
        result flag
        target_configuration, a list of 6 doubles

        """
        v = limb.state.cartesianDriveV
        w = limb.state.cartesianDriveW
        amount = self.dt * limb.state.driveSpeedAdjustment
        target_transform = (so3.mul(so3.from_moment(vectorops.mul(w,amount)),\
            limb.state.driveTransform[0]),vectorops.add(\
            limb.state.driveTransform[1],vectorops.mul(v,amount)))

        #print("target_transform:",target_transform)
        ###debugging

        #joint position limits from the joint speed limit

        if self.mode == 'Kinematic':
            C = self.dt
        elif self.mode == 'Physical':
            C = self.dt*30
        joint_upper_limits = vectorops.add(limb.state.sensedq,vectorops.mul(\
            TRINAConfig.limb_velocity_limits,C))
        joint_lower_limits = vectorops.add(limb.state.sensedq,vectorops.mul(\
            TRINAConfig.limb_velocity_limits,-C))

        # NOTE: REQUESTING DOCUMENTATION
        if limb.state.cartesianMode == 0:
            goal = ik.objective(limb.EE_link,R=target_transform[0],\
                t = vectorops.sub(target_transform[1],so3.apply(target_transform[0],limb.state.toolCenter)))
        elif limb.state.cartesianMode == 1:
            goal = ik.objective(limb.EE_link,local = [0,0,0], \
                world = vectorops.sub(target_transform[1],so3.apply(target_transform[0],limb.state.toolCenter)))

        initialConfig = self.robot_model.getConfig()
        # PATRICK - Change this back
        # res = ik.solve_nearby(goal,maxDeviation=0.5,activeDofs = limb.active_dofs,tol=0.000001)
        res = ik.solve_nearby(goal,maxDeviation=0.5,activeDofs = limb.active_dofs,tol=0.001, numRestarts=10)

        failFlag = False
        if res:
            # Converting to numpy array to use slice-by-list.
            if not self._arm_is_in_limit(np.array(self.robot_model.getConfig())[limb.active_dofs],
                        joint_upper_limits, joint_lower_limits):
                #print('failed because not in limit')
                failFlag = True
        else:
            failFlag = True
            #print("motion.controlLoop():IK solution not found")
        if failFlag:
            # NOTE: Is it better to decay by binary search here?
            limb.state.driveSpeedAdjustment = limb.state.driveSpeedAdjustment - 0.1
            if limb.state.driveSpeedAdjustment < 0.001:
                # self.left_limb_state.cartesianDrive = False

                logger.error('CartesianDrive IK has failed completely,exited..')
                print("motion.controlLoop():CartesianDrive IK has failed completely,exited..")

                jacobian = np.array(limb.EE_link.getJacobian([0,0,0]))
                del_theta = np.linalg.lstsq(jacobian, np.array(se3.error(target_transform, current_transform)))[0]
                del_theta = 0.1 * del_theta / np.linalg.norm(del_theta)
                target_config = initialConfig[:]
                for i, ind in enumerate(limb.active_dofs):
                    # NOTE: What if target config is out of bounds?
                    target_config[ind] += del_theta[ind]

                return 2, target_config # 2 means success, maybe this should have a different return signal.
            else:
                logger.warning('CartesianDrive IK has failed partially')
                #print("motion.controlLoop():CartesianDrive IK has failed, next trying: ",\
                #   limb.state.driveSpeedAdjustment)
                return 1,0 # 1 means the IK has failed partially and we should do this again
        else:
            #print('success!')
            logger.info('CartesianDrive IK has succeeded')
            # Converting to numpy array to use slice-by-list.
            target_config = np.array(self.robot_model.getConfig())[limb.active_dofs]
            limb.state.driveTransform = target_transform
            # NOTE: This is bug-prone and can lead to overshoot due to numerical error!
            if limb.state.driveSpeedAdjustment < 1:
                limb.state.driveSpeedAdjustment += 0.1

        self.robot_model.setConfig(initialConfig)
        
        # NOTE: LimbController only takes python floats!!! THIS IS DANGEROUS!
        return 2,target_config.tolist() #2 means success..

    def _simulate_2(self,wrench,m_inv,K,B,T_curr,x_dot_curr,T_g,x_dot_g,dt):
        """
        Simulate a mass spring damper under external load, semi-implicit Euler integration

        Parameters:
        -----------------
        m_inv: 6x6 numpp array,inverse of the mass matrix
        K: 6x6 numpy array, spring constant matrix
        B, 6x6 numpy array, damping constant matrix
        T_curr: rigid transform, current transform of the mass
        x_dot_curr: lost of 6, curent speed
        T_g: rigid transform, target transform
        x_dot_g: a list of 6
        dt:simulation dt

        Return:
        -----------------
        x,v: list of 62.0
        """

        e = se3.error(T_g,T_curr)
        e = np.array(e[3:6] + e[0:3])
        x_dot_g = np.array(x_dot_g)
        v = np.array(x_dot_curr)
        e_dot = x_dot_g - v
        wrench_total = wrench + np.dot(K,e) + np.dot(B,e_dot)
        a = np.dot(m_inv,wrench_total)
        #limit maximum acceleration
        # a = np.clip(a,[-1,-1,-1,-4,-4,-4],[1,1,1,4,4,4])
        a = np.clip(a,[-0.3]*6,[0.3]*6)
        # print('external wrench:',wrench)
        # print('spring wrench:',np.dot(K,e))
        # print('mass v:',v)
        # print('damping wrench:',np.dot(B,e_dot))
        # print('error:',e)
        # print('simulated accel:',a)
        v = v + a*dt
        #limit maximum velocity
        v = np.clip(v,[-1,-1,-1,-1,-1,-1],[1,1,1,1,1,1])
        dx = v*dt
        # print('dx',dx)
        T = se3.mul((so3.from_moment(dx[3:6]),dx[0:3]),T_curr)
        return T,v.tolist()

    def _simulate(self,wrench,A_mat,m_inv,K,B,T_curr,x_dot_curr,T_g,x_dot_g,dt):
        e = se3.error(T_g,T_curr)
        e = np.array(e[3:6] + e[0:3])
        x_dot_g = np.array(x_dot_g)
        v = np.array(x_dot_curr)
        e_dot = x_dot_g - v
        affine_term = (np.concatenate((e, e_dot)) 
            + np.concatenate((np.zeros(6), self.dt * m_inv @ wrench)))
        e_next = np.linalg.solve(A_mat, affine_term)
        # e_next = A_mat @ np.concatenate((e, e_dot)) + affine_term
        x_next = se3.mul(T_g, (so3.from_moment(e_next[3:6]), e_next[0:3]))
        v_next = x_dot_g - e_next[6:]
        print("error", e)
        print("e_next", e_next)

        print("v_next", v_next)
        print("del v", self.dt * m_inv @ wrench)
        print("------------------------------------------")
        return x_next, v_next

    def _impedance_drive(self, limb):
        """Calculate the next goal for impedance control
        Parameters:
        --------------
        limb: The limb to drive

        Return:
        --------------
        Result flag
        target_config : list of doubles, the target limb config
        """
        state = limb.state
        wrench = limb.sensedEEWrench(frame = 'global')
        #if force too big, backup a bit and stop
        stop = False
        if vectorops.norm_L2(wrench[0:3]) > 60:
            stop = True
            print('Max force exceeded')
        if stop:
            # TODO: Dangerous (tool center)
            TEE = limb.sensedEETransform()
            #move back 20 mm
            T = (TEE[0],vectorops.add(TEE[1],vectorops.mul(vectorops.unit(wrench[0:3]),0.02)))
        else:
            # The hack.
            wrench = np.array(wrench)
            displace_wrench = np.array(wrench[:3])
            # Force magnitude.
            mag = np.linalg.norm(displace_wrench)
            old_wrench = state.prev_wrench[:3]

            effective_b = np.copy(state.B)

            START_THRESHOLD = 1.5
            STOP_THRESHOLD = 4

            if state.increaseB:
                if mag < STOP_THRESHOLD:
                    state.increaseB = False
            elif np.linalg.norm(displace_wrench - old_wrench) > START_THRESHOLD and mag > 0.0:
                state.increaseB = True

            # print(f"DAMPING STATE: {[state.increaseB,mag]}")
            # if state.increaseB:
            #     effective_b *= 20

            for i in range(6):
                if state.deadband[i] > 0:
                    if math.fabs(wrench[i]) < state.deadband[i]:
                        wrench[i] = 0
            # start = time.monotonic()
            N = 20
            for i in range(N):
                # state.T_mass, state.x_dot_mass = self._simulate(wrench = wrench, A_mat=state.A, m_inv = state.Minv,\
                #     K = state.K,B = effective_b,T_curr = state.T_mass,x_dot_curr = state.x_dot_mass,\
                #     T_g = state.T_g,x_dot_g = state.x_dot_g,dt = self.dt/N) 
                state.T_mass, state.x_dot_mass = self._simulate_2(wrench = wrench, m_inv = state.Minv,\
                    K = state.K,B = effective_b,T_curr = state.T_mass,x_dot_curr = state.x_dot_mass,\
                    T_g = state.T_g,x_dot_g = state.x_dot_g,dt = self.dt/N) 
            # print("Time: ", time.monotonic() - start)
            state.counter += 1

            state.prev_wrench = np.array(wrench)

            #orthogonalize the rotation matrix
            # if state.counter % 100 == 0:
            #     state.T_mass = (so3.from_moment(so3.moment(state.T_mass[0])),state.T_mass[1])
            T = state.T_mass

        goal = ik.objective(limb.EE_link,R=T[0],\
            t = vectorops.sub(T[1],so3.apply(T[0],state.toolCenter)))

        initialConfig = self.robot_model.getConfig()
        res = ik.solve_nearby(goal,maxDeviation=0.5,activeDofs = limb.active_dofs,tol=0.0001)
        if not res:
            logger.error('ImpedanceDrive IK has failed y,exited..')
            print("motion.controlLoop():ImpedanceDrive IK has failed, exited this mode")
            return 0,0
        else:
            target_config = np.array(self.robot_model.getConfig())[limb.active_dofs]
        self.robot_model.setConfig(initialConfig)

        if stop:
            # NOTE: LimbController only takes python floats!!! THIS IS DANGEROUS
            return 2,target_config.tolist()
        else:
            return 1,target_config.tolist()

    def _get_klampt_q(self,left_limb = [],right_limb = []):
        if len(left_limb):
            return TRINAConfig.get_klampt_model_q(self.codename,left_limb = left_limb, right_limb = self.right_limb.state.sensedq)
        elif len(right_limb):
            return TRINAConfig.get_klampt_model_q(self.codename,left_limb = self.left_limb.state.sensedq, right_limb = right_limb)

# def imp_sim(t, x):


if __name__=="__main__":

    ###Read the current position ###
    robot = Motion(mode = 'Physical',components = ['left_limb','right_limb'],codename = "bubonic")
    robot.startup()
    time.sleep(0.05)
    # robot.setLeftLimbPositionLinear([-4.02248,0.1441026,1.58109,-0.254,0.9090495,0.46262],30)
    # print(robot.getKlamptSensedPosition())
    # with open('tmp.txt', 'a') as f:
    #     f.write(f"\n{str(robot.getKlamptSensedPosition())}")
    left_pos = robot.sensedLeftEETransform()
    K = np.diag([200.0, 200.0, 200.0, 1000, 1000, 1000])
    # K = np.zeros((6,6))
    # K[3:6,3:6] = np.eye(3)*1000

    M = 1*np.eye(6)#*5.0
    M[3,3] = 1.0
    M[4,4] = 1.0
    M[5,5] = 1.0

    B = 3.0*np.sqrt(4.0*np.dot(M,K))
    # B = 30*np.eye(6)
    # B[3:6,3:6] = 0.1*B[3:6,3:6]
    # self.B[3:6,3:6] = self.B[3:6,3:6]*2.0
    # self.M = np.diag((2,2,2,1,1,1))
    # self.B = np.sqrt(32 * self.K *ABSOLUTE self.M)
    # K = K.tolist()
    # M = M.tolist()
    # B = B.tolist()
    robot.setLeftEETransformImpedance(left_pos, K, M, B)
    # robot.setLeftEEInertialTransform(left_pos, 0.1)
    print("Holding position")
    while True:
        # print('{:2.3f}\t{:2.3f}\t{:2.3f}\t{:2.3f}\t{:2.3f}\t{:2.3f}'.format(*robot.sensedLeftEEWrench(frame='global')))
        time.sleep(0.01)
    robot.shutdown()

