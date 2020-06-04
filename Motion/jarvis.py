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
import trina_modules
import sys, inspect
import command_server

class Jarvis:

    def __init__(self):
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.command_server = CommandServer()

    def shutdown(self):
        self.command_server.shutdown()

	def sensedBaseVelocity(self):
		return self.server["ROBOT_STATE"]["Velocity"]["Base"]

	def sensedLeftLimbVelocity(self):
		return self.server["ROBOT_STATE"]["Velocity"]["LeftArm"]

	def sensedRightLimbVelocity(self):
		return self.server["ROBOT_STATE"]["Velocity"]["RightArm"]

	def sensedBasePosition(self):
		return self.server["ROBOT_STATE"]["Position"]["Base"]

	def sensedTorsoPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["Torso"]

	def sensedLeftLimbPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["LeftArm"]

	def sensedRightLimbPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["RightArm"]

	def sensedBasePosition(self):
		return self.server["ROBOT_STATE"]["Position"]["Base"]

    def sensedLeftGripperPosition(self):
		return self.server["ROBOT_STATE"]["Position"]["LeftGripper"]

	def setLeftLimbPosition(self,q):
        command = send_command(Motion.setLeftLimbPosition,q)
        current_list = server['ROBOT_COMMAND']['3']
        server['ROBOT_COMMAND']['3'] = current_list.add(command)
        
    def send_command(command,*args):
        final_string = str(command)+ '('
        for index,arg in enumerate(args):
            if(index != len(args)-1):
                final_string += '{},'
            else:
                final_string += '{}'
        final_string = (final_string + ')')
        final_string = final_string.format(*args)
        return final_string

if __name__=="__main__":
    server = Jarvis()
