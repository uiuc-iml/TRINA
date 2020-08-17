import logging
#logging.basicConfig(level=logging.INFO)
import time
import numpy as np
import cv2
import pyrealsense as pyrs
from pyrealsense.constants import rs_option

from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
import math
import random

import matplotlib.pyplot as plt
from scipy import signal
from copy import deepcopy

num_pic = 18

def take_pictures(camera,robot,arm,configurations):
    EE_transforms = []
    for config in configurations:
        if arm == 'left':
            robot.setLeftLimbPositionLinear(config,5)
        elif arm == 'right':
            robot.setRightLimbPositionLinear(config,5)
        time.sleep(5)
        
        if arm == 'left':
            EE_transforms.append(robot.sensedLeftEETransform())
        elif arm == 'right':
            EE_transforms.append(robot.sensedRightEETransform())
        ##get CAD and pointclouds 
        cv2.imwrite('calbration_pic_'+str(i)+'.png',cad)
        cv2.imshow('color',cad)
        cv2.waitKey(500)
        data=open(data_path+'calibration_pt_'+str(i)+'.txt','w')
        #TODO: make a better way to save the pointcloud?
        #Can I use the sensing module to save these?
        for y in range(480):
            for x in range(640):
                data.write(str(p[y][x][0])+' '+str(p[y][x][1])+' '+str(p[y][x][2])+'\n')
        data.close()
        time.sleep(0.5)
        print('point %d'%i)
    return EE_transforms
if __name__ == "__main__":
    take_pictures()