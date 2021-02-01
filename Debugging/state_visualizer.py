#!/usr/local/lib/python2.7
import sys
import time,math
from klampt import vis
import numpy as np
import os
import pdb
from klampt.math import so3

try:
    import trina
except ImportError:
    sys.path.append(os.path.expanduser("~/TRINA"))
    import trina
from Motion.motion_client import MotionClient

dt = 1.0/30.0

robot = MotionClient(address=trina.settings.motion_server_addr())
res = robot.startup()
if not res:
    raise RuntimeError("Motion server not running?")

#add more visualization items here under the format {key:STATE_SERVER_PATH}
visItems = {
    
}

#add more plot items here under the format {plot: items} where items is a dict {key:STATE_SERVER_PATH}
plotItems = {
    "Controllers" : {
        "left_joystick_x" : ['UI_STATE','controllerButtonState','leftController','thumbstickMovement',0],
        "left_joystick_y" : ['UI_STATE','controllerButtonState','leftController','thumbstickMovement',1],
        "right_joystick_x" : ['UI_STATE','controllerButtonState','rightController','thumbstickMovement',0],
        "right_joystick_y" : ['UI_STATE','controllerButtonState','rightController','thumbstickMovement',1],
    }
}

server = trina.state_server.StateServer()
# robot.startServer(mode = 'Kinematic', components = [])


world = trina.setup.simulation_world_load()
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

for plotName,plotPaths in plotItems.items():
    vis.addPlot(plotName)

t0 = time.time()
while vis.shown():
    vis.lock()
    #temp: try sending a motion
    sensed_position = server["ROBOT_STATE"]["Position"]["Robotq"].read()
    vis_robot.setConfig(sensed_position)

    #plot various things
    for plotName,plotPaths in plotItems.items():
        for item,path in plotPaths.items():
            try:
                value = server.get(path)
            except Exception as e:
                print("Error",e,"while getting path",path)
                import traceback
                traceback.print_exc()
                continue
            if hasattr(value,'__iter__'):
                for i,v in enumerate(value):
                    vis.logPlot(plotName,'{}[{}]'.format(v,i),v)
            else:
                vis.logPlot(plotName,item,value)
        
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
    #vp = vis.getViewport()
    #camera = vp.camera
    #camera.set_matrix([final_view,final_dist]) 

    # pdb.set_trace()
    vis.unlock()

    time.sleep(dt)

robot.stopMotion()
robot.resumeMotion()

vis.kill()