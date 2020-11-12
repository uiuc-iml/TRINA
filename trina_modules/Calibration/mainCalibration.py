#Calibration functions for TRINA, without using Jarvis. Only dependent on motion and sensor modules.
#In this file, both the camera and URDF will be calibrated.
import os
# from klampt.model import trajectory as klamptTraj
from klampt.io import loader
from calibrationLogger import CalibrationLogger
import cv2
def detectMarker1(pic):
    """
    Parameters:
    -------------------
    A picture containing marker1, which consists of 3 markers defining a frame.
    Assume the 3 markers are arruco markers with different IDs.

    The markers are 

    1

    2    3
    

    Return:
    ------------------
    A rigid transform, in camera coordinate
    """
    IDs = [1,2,3]
    marker_sz = 

    T_marker = []
    return T_marker

def extractData1(save_path,cameras):
    """
    Extract data of marker1, the static marker that defines a transform
    
    Return:
    -----------------
    markers: A list of lists, where each list contains the detected transforms in a camera
    qs: A list of lists, where each list contains the robot configurations at the pictures
    """

    markers = []
    qs = []
    for cn in cameras:
        for il,l in enumerate(open(save_path+'/robot_states.txt','r').readlines()):
            frame = cv2.imread(folder+"/color_overhead-%05d.jpg"%il)
        pl,pr,pt=detectMarker1(frame,markerSz,markerSzTable,dict,dictTable,lMarker,rMarker,mtx,dist)
        if pl is not None or pr is not None:
            
    return


def takePictures(traj_path,save_path,cameras,motion_address,codename):
    """
    Read the trajectory for taking the video and execute it. The pictures are recorded with the server automatically, to the save path
    
    Parameter:
    -------------------
    path to the trajectory file

    Return:
    -------------------
    None.

    """
    if not os.path.isdir(save_path):
        os.mkdir(save_path)
    ref_traj = loader.loadTrajectory(traj_path)
    logger = CalibrationLogger(cameras,motion_address,codename)
    logger.collect(ref_traj,save_path)    
    return 0


def mainCalibration(traj_path,save_path,calibration_type,cameras,links,motion_address,codename):
    """
    There are several different types of calibration. 
    1) URDF calibration, the arm mountings and 2 wrist mounted cameras will be calibrated. 
    The rest here assumes that the URDF has already been calibrated.
    2) Wrist mounted camera calibration. One or multiple cameras are calibrated. 
    3) Fixed camera calibration, where one or multiple fixed cameras will be calibrated.  

    calibration_type
    """
    if calibration_type == 'URDF':
        #The cameras should be ['left_realsense','right_realsense'] here.
        takePictures(traj_path,save_path,cameras,motion_address,codename)
        Tmarkers,configs = extractData(save_path,cameras)
        URDFCalibration(Tmarkers,configs,URDF_path,links)
