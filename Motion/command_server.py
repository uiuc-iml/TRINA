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
from multiprocessing import Pool, TimeoutError
from Modules import *

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

        # create the list of threads

        self.threads = []
        for i in range(len(self.modules)):
            t = threading.Thread(name = self.module[i], target=activate, args = (self.modules[i],))
            threads.append(t)
            t.start()
        # each thread will be assigned to start each module

        stateRecieverThread = threading.Thread(target=self.stateReciever)
        stateRecieverThread.start()
        commandRecieverThread = threading.Thread(target=self.commandReciever)
        commandRecieverThread.start()
        moduleMonitorThread = threading.Thread(target=self.moduleMonitor)
        moduleMonitorThread.start()

#this is place holder for moduleMonitor
    def activate(name):
        while not self.shut_down_flag:
            time.sleep(0.1)

    def stateReciever(self):
        pos_left = {}
        pos_right = {}
        pos_base = {}
        pos_left_gripper = {}
        pos_right_gripper = {}
        pos_torso = {}
        vel_base = {}
        vel_right = {}
        vel_left = {}
        loopStartTime = time.time()
        while not self.shut_down_flag:
            self.robot_state = self.server['ROBOT_STATE'].read()
            self.startup = False
            if(self.left_limb_active):
                pos_left = self.robot.sensedLeftEETransform()
                print("left position")
                vel_left = self.robot.sensedLeftEEVelcocity()
                print("left velocity")
            if(self.right_limb_active):
                pos_right = self.robot.sensedRightEETransform()
                print("right position")
                vel_right = self.robot.sensedRightEEVelcocity()
                print("right velocity")
            if(self.base_active):
                pos_base = self.robot.sensedBasePosition()
                print("base position")
                vel_base = self.robot.sensedBaseVelocity()
                print("base velocity")
            if(self.left_gripper_active):
                pos_left_gripper = self.robot.sensedLeftGripperPosition()
                print("left gripper position")
            if(self.right_gripper_active):
                pos_right_gripper = self.robot.sensedRightGripperPosition()
                print("right gripper position")
            if(self.torso_active):
                pos_torso = self.robot.sensedTorsoPosition()
                print("torso position")

            UI_state = self.server["UI_STATE"].read()
            # build the state.
            self.server["ROBOT_STATE"] = {
                                    "Position" : {
                                        "leftArm" : pos_left,
                                        "rightArm" : pos_right,
                                        "base" : pos_base,
                                        "torso": pos_torso,
                                        "leftGripper" : pos_left_gripper,
                                        "rightGripper" : pos_right_gripper,
                                        },
                                    "Velocity" : {
                                        "leftArm" : vel_left,
                                        "rightArm" : vel_right,
                                        "base" : vel_base,
                                    },
                                    }
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
            elapsedTime = time.time() - loopStartTime
            for i in range(5):
                if (len(self.server['ROBOT_COMMAND'][str(i)]) != 0){
                    commandsList = self.server['ROBOT_COMMAND'][str(i)]
                    run(commandList[0])
                    commandList.pop(0)
                    break;
                }
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    def run(command):
        try:
            exec(command)
        finally:
            print("command recieved was " + command)

#0 -> dead
#1 -> healthy
    def moduleMonitor(self):
        while not self.shut_down_flag:
            loopStartTime = time.time()
            for module in self.modules:
                moduleStatus = self.server[module]["Status"].read()
                if moduleStatus == 0:
                    print("Module is dead")
                    #restart
                    print(module ++ " restarted")
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
