import time,math,datetime
import threading
from motion_client import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import numpy as np
from scipy.spatial.transform import Rotation as R
import os,csv,sys
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback
from UIController_reem import UIController

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
        self.init_headset_orientation = {}
        self.startup = True
        self.robot_active = True
        self.shut_down_flag = False
        res = self.robot.startup()
        if not res:
            return

        stateRecieverThread = threading.Thread(target=self.stateReciever)
        stateRecieverThread.start()
        commandRecieverThread = threading.Thread(target=self.commandReciever)
        commandRecieverThread.start()


    def stateReciever(self):
        while(True):
            if(self.startup == True & (self.server["ROBOT_STATE"].read()!=0)):
                self.init_robot_state = self.server['ROBOT_STATE'].read()
                self.startup = False
                # if(self.is_robot_active())
                    ####

        loopStartTime = time.time()
        while not self.shut_down_flag:

            self.UI_state = self.server['ROBOT_STATE'].read()
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

        return (left_limb_active or right_limb_active or base_active or left_gripper_active or right_gripper_active or torso_active)

if __name__=="__main__":
    server = CommandServer()
