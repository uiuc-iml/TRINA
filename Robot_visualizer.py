#!/usr/local/lib/python2.7
import sys
import time,math
from klampt import vis
from klampt import WorldModel
from klampt.model.trajectory import Trajectory
import threading
from Motion.motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import pickle
from pdb import set_trace
import time,math
from numpy import linalg as LA
import numpy as np
import os
import datetime
import csv 
import time
from threading import Thread
import time
import sys
import json
import pdb
from klampt.math import so3

# robot_ip = 'http://192.168.0.143:8080'
# robot_ip = 'http://172.16.241.141:8080'
robot_ip = 'http://localhost:8080'
ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

roomname = "The Lobby"
zonename = "BasicExamples"
userId=0
roomId=-1
is_closed=0
dt = 1.0/30.0

robot = MotionClient(address = robot_ip)
# robot.startServer(mode = 'Kinematic', components = [])

def visualUpdateLoop():

    while True:
        # try:
        # pdb.set_trace()

        vis.lock()
        sensed_position = robot.getKlamptSensedPosition()
        # print(len(sensed_position))
        vis_robot.setConfig(sensed_position)
        ## end effector is 42
        # Adding the third person perspective. First, select a link that is aligned with the base
        EndLink = vis_robot.link(3)           # This link number should be the end effector link number
        # get that link's T
        Tlink = EndLink.getTransform()

        # EE_link = vis_robot.link(41)
        # transform = EE_link.getTransform()
        # vis.add("Frame",transform)
        # Do transforms to get it into Klampt format
        rot_link = so3.from_matrix(so3.matrix(Tlink[0]))
        #add view angle of 45 degrees down
        rot_view = so3.from_matrix(so3.matrix(so3.from_axis_angle([[0,1,0],-np.pi/4])))
        inter_view = so3.mul(rot_link,rot_view)
        #add a view angle to make sure the camera is upright
        rot_view_2 = so3.from_matrix(so3.matrix(so3.from_axis_angle([[0,0,1],-np.pi/2])))
        final_view = so3.mul(inter_view,rot_view_2)
        #Make sure the camera arm moves together with the robot by rotating it along its axis.
        intermediate_dist = so3.apply(Tlink[0],[-3.05,0,3.6])
        # final_dist = (np.array(Tlink[1]) + np.array([-3.05,0,3.6])).tolist()
        final_dist = (np.array(Tlink[1]) + np.array(intermediate_dist)).tolist()
        
        # final_dist = so3.apply(final_view,new_dist)
        # apply the new camera view
        vp = vis.getViewport()
        camera = vp.camera
        camera.set_matrix([final_view,final_dist]) 

        # pdb.set_trace()
        vis.unlock()

        time.sleep(dt)


res = robot.startup()

world = WorldModel()
res = world.readFile(model_name)
if not res:
    raise RuntimeError("Unable to load Klamp't model")
vis_robot = world.robot(0)

vis.add("world",world)
vis.show()



def translate_camera(linknum):
    vis.lock()
    link = vis_robot.link(linknum)           # This link number should be the end effector link number
    Tlink = link.getTransform()
    camera.set_matrix(Tlink)
    vis.unlock()
    return Tlink
# while(True):#     print('\n\n {} \n\n'.format(i))tra
#     try:
#         vis.lock()
#         link = vis_robot.link(i)           # This link number should be the end effector link number
#         Tlink = link.getTransform()
#         camera.set_matrix(Tlink)
#         vis.unlock()
#         time.sleep(1)
#         i+=1
#         print(i)
#     except:
#         print('error occured!!!!!')
#         break
while True:
    time.sleep(0.5)
    visualUpdateLoop()
