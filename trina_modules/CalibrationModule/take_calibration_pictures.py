import logging
#logging.basicConfig(level=logging.INFO)
import time
import numpy as np
import cv2
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
import math
import random

import matplotlib.pyplot as plt
from scipy import signal
from copy import deepcopy

def take_pictures(camera,robot,arm,configurations):
    if camera:
        pass
    else:
        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        config = rs.config()
        if arm == 'right':
            config.enable_device("620202003661".encode('utf-8'))
        elif arm == 'fixed':
            config.enable_device("620201003873".encode('utf-8'))

        config.enable_stream(
            rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(
            rs.stream.color, 640, 480, rs.format.rgb8, 30)
        align_to = rs.stream.color
        align = rs.align(align_to)
        pc = rs.pointcloud()
        # Start streaming
        pipeline.start(config)

        time.sleep(3)
        print '---------------------camera initiated -----------------------------'



    EE_transforms = []
    i = 0
    for config in configurations:
        t = 3.0
        if arm == 'left':
            robot.setLeftLimbPositionLinear(config,t)
        elif arm == 'right':
            robot.setRightLimbPositionLinear(config,t)
        elif arm == 'fixed':
            robot.setRightLimbPositionLinear(config,t)
        time.sleep(t+0.5)
        

        if arm == 'left':
            EE_transforms.append(robot.sensedLeftEETransform())
            while np.linalg.norm(np.array(robot.sensedLeftEEVelocity()[0])) > 0.001:
                time.sleep(0.1)
        elif arm == 'right' or arm == 'fixed':
            EE_transforms.append(robot.sensedRightEETransform())
            while np.linalg.norm(np.array(robot.sensedRightEEVelocity()[0])) > 0.001:
                time.sleep(0.1)

        if camera:
            pass
        else:
            # Wait for the next set of frames from the camera
            frames = pipeline.wait_for_frames()
            # Fetch color and depth frames and align them
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                print("Data Not Available at the moment")
                return None
            # Tell pointcloud object to map to this color frame
            pc.map_to(color_frame)
            # Generate the pointcloud and texture mappings
            points = pc.calculate(depth_frame)
            vtx = np.asarray(points.get_vertices())
            pure_point_cloud = np.zeros((640*480, 3))
            pure_point_cloud[:, 0] = -vtx['f0']
            pure_point_cloud[:, 1] = -vtx['f1']
            pure_point_cloud[:, 2] = -vtx['f2']

            cad = cv2.cvtColor(np.asarray(color_frame.get_data()), cv2.COLOR_RGB2BGR)
            p = pure_point_cloud
        ##get CAD and pointclouds 
        cv2.imwrite('calbration_pic_'+str(i)+'.png',cad)
        data=open('calibration_pt_'+str(i)+'.txt','w')
        #TODO: make a better way to save the pointcloud?
        #Can I use the sensing module to save these?
        for y in range(480):
            for x in range(640):
                # data.write(str(p[y][x][0])+' '+str(p[y][x][1])+' '+str(p[y][x][2])+'\n')
                data.write(str(p[y*640+x,0])+' '+str(p[y*640+x,1])+' '+str(p[y*640+x,2])+'\n')
        data.close()
        time.sleep(0.5)
        print('point %d'%i)
        i += 1
    return EE_transforms

if __name__ == "__main__":
    take_pictures()