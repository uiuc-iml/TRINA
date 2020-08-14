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

def controller_2_klampt(robot,controllerQ):
    qOrig=robot.getConfig()
    q=[v for v in qOrig]
    for i in range(6):
        q[i+1]=controllerQ[i]
    return q

def klampt_2_controller(robotQ):

    temp=robotQ[1:7]
    temp.append(0)
    return temp

def take_pictures(camera,robot,arm,configurations):
    for config in configurations:
        if arm == 'left':
            robot.setLeftLimbPositionLinear(config,5)
        elif arm == 'right':
            robot.setRightLimbPositionLinear(config,5)

        ##get CAD and pointclouds 
        cv2.imwrite('calbration_pic_'+str(i)+'.png',cad)
        cv2.imshow('color',cad)
        cv2.waitKey(500)
        data=open(data_path+'calibration_pt_'+str(i)+'.txt','w')
        #TODO: make a better way to save the pointcloud?
        for y in range(480):
            for x in range(640):
                data.write(str(p[y][x][0])+' '+str(p[y][x][1])+' '+str(p[y][x][2])+'\n')
        data.close()
        time.sleep(0.5)
        print('point %d'%i)

if __name__ == "__main__":
    take_pictures()