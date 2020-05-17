import time,math,datetime
import threading
from motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import numpy as np
from scipy.spatial.transform import Rotation as R
import os,csv,sys
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback

robot_ip = 'http://localhost:8080'

ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

class CommandServer:

    def __init__(self):
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["ROBOT_STATE"] = 0
        self.server["ROBOT_COMMAND"] = 0
        self.mode = 'Kinematic'
        self.components = ['base','left_limb','right_limb','left_gripper']
        self.init_robot_state = {}
        self.dt = 0.001
        self.robot = MotionClient(address = robot_ip)
        self.controller = UIController()
        self.robot.startServer(mode = self.mode, components = self.components,codename = 'seed')
        self.robot_state = {}
        self.robot_command = {}
        self.modules = ["Wipe", "UIModule", "Motion"]
        self.init_headset_orientation = {}
        self.startup = True
        self.robot_active = True
        self.shut_down_flag = False
        self.left_limb_active = ('left_limb' in self.components)
        self.right_limb_active = ('right_limb' in self.components)
        self.base_active = ('base' in self.components)
        self.left_gripper_active = ('left_gripper' in self.components)
        self.right_gripper_active = ('right_gripper' in self.components)
        self.torso_active = ('torso' in self.components)
        res = self.robot.startup()
        if not res:
            return

        stateRecieverThread = threading.Thread(target=self.stateReciever)
        stateRecieverThread.start()
        commandRecieverThread = threading.Thread(target=self.commandReciever)
        commandRecieverThread.start()
        modulesRecieverThread = threading.Thread(target=self.moduleReciever)
        modulesRecieverThread.start()

    def stateReciever(self):
        while(True):
            if(self.startup == True & (self.server["ROBOT_STATE"].read()!=0)):
                self.init_robot_state = self.server['ROBOT_STATE'].read()
                self.startup = False
                if(self.left_limb_active):
                    self.init_pos_left = self.robot.sensedLeftEETransform()
                if(self.right_limb_active):
                    self.init_pos_right = self.robot.sensedRightEETransform()
                if(self.base_active):
                    # self.init_pos_base = self.robot.
                    print("base position")
                if(self.left_gripper_active):
                    # self.init_pos_left_gripper = self.robot.
                    print("left gripper position")
                if(self.right_gripper_active):
                    # self.init_pos_right_gripper = self.robot.
                    print("right gripper position")
                if(self.torso_active):
                    # self.init_pos_torso = self.robot.
                    print("torso position")
                self.init_headset_orientation = self.treat_headset_orientation(self.init_UI_state['headSetPositionState']['deviceRotation'])
        loopStartTime = time.time()
        while not self.shut_down_flag:
            if(self.left_limb_active):
                self.init_pos_left = self.robot.sensedLeftEETransform()
            if(self.right_limb_active):
                self.init_pos_right = self.robot.sensedRightEETransform()
            if(self.base_active):
                # self.init_pos_base = self.robot.
                print("base position")
            if(self.left_gripper_active):
                # self.init_pos_left_gripper = self.robot.
                print("left gripper position")
            if(self.right_gripper_active):
                # self.init_pos_right_gripper = self.robot.
                print("right gripper position")
            if(self.torso_active):
                # self.init_pos_torso = self.robot.
                print("torso position")
            # build the state.
            self.server["ROBOT_STATE"] = {}
            self.UI_state = self.server['UI_STATE'].read()
            ################
            elapsedTime = time.time() - loopStartTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    def commandReciever(self):
        while not self.shut_down_flag:
            loopStartTime = time.time()
            self.robot_command = self.server['ROBOT_COMMAND'].read()
            #execute the commands
            elapsedTime = time.time() - loopStartTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    def moduleReciever(self):
        while not self.shut_down_flag:
            loopStartTime = time.time()

            for module in self.modules:
                moduleStatus = self.server[module]["Status"].read()
                if moduleStatus == 0:
                    #restart
                    print(module ++ " restarted")
                elif moduleStatus == 1:
                    #turn off
                    print(module ++ " turned off")
                elif moduleStatus == 2:
                    #turn on
                    print(module ++ " turned on")
                else:
                    pass



            elapsedTime = time.time() - loopStartTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    def sigint_handler(self, signum, frame):
        """ Catch Ctrl+C tp shutdown the robot

        """
        assert(signum == signal.SIGINT)
        logger.warning('SIGINT caught...shutting down the api!')
        print("SIGINT caught...shutting down the api!")
        self.shutdown()

    def shutdown():
        #send shutdown to all modules
        self.shut_down_flag = True
        return 0


    def is_robot_active(self):
        left_limb_active = ('left_limb' in self.components)
        right_limb_active = ('right_limb' in self.components)
        base_active = ('base' in self.components)
        left_gripper_active = ('left_gripper' in self.components)
        right_gripper_active = ('right_gripper' in self.components)
        torso_active = ('torso' in self.components)

        return (left_limb_active and right_limb_active and base_active and left_gripper_active and right_gripper_active and torso_active)


    def setRobotToDefault(self):
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        print('right_Untucked',rightUntuckedConfig)
        if('left_limb' in self.components):
            self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,2)
        if('right_limb' in self.components):
            self.robot.setRightLimbPositionLinear(rightUntuckedConfig,2)

if __name__=="__main__":
    server = CommandServer()
