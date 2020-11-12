#Calibration functions for TRINA, without using Jarvis. Only dependent on motion and sensor modules.
#In this file, both the camera and URDF will be calibrated.
import os
# from klampt.model import trajectory as klamptTraj
from klampt.io import loader
from calibrationLogger import CalibrationLogger
import cv2
import cv2.aruco as aruco
from klampt.math import vectorops as vo

def detectAruco(pic,marker_sz,IDs,dictionary)
    if cn == 'realsense_left':
        pass
        #TODO: add realsense camera intrinsics
    elif cn == 'realsense_right':
        pass
    elif cn == 'zed_overhead':
        gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
        if frame.shape==(1920,1080):
            fx=1055.28
            fy=1054.69
            cx=982.61
            cy=555.138
            k1=-0.0426844
            k2=0.0117943
            k3=-0.00548354
            p1=0.000242741
            p2=-0.000475926
        else:
            fx=1055.28
            fy=1054.69
            cx=1126.61
            cy=636.138
            k1=-0.0426844
            k2=0.0117943
            k3=-0.00548354
            p1=0.000242741
            p2=-0.000475926
        mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
        dist = np.array([k1,k2,p1,p2,k3])

    parameters = aruco.DetectorParameters_create()
    #TODO: adjust these
    aruco_dict = aruco.Dictionary_get(dictionary)
    # aruco_dict_table = aruco.Dictionary_get(aruco.DICT_7X7_50)

    corners, detected_ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, \
                                                            cameraMatrix=mtx, distCoeff=dist)        
    #add the detected marker position 
    ps = []
    for target_id in IDs:
        if target_id in detected_ids:
            #get the position
            rvec,tvec,_=aruco.estimatePoseSingleMarkers(c,marker_sz,mtx,dist)
            ps.append(tvec[0,0,:].tolist())
        else:
            ps.append([])
    return ps

def detectMarker1(pic,cn):
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
    marker_sz = 0.05
    dictionary = aruco.DICT_ARUCO_ORIGINAL
    ps = detectAruco(pic,marker_sz,IDs,dictionary)

    if [] in ps:
        print('marker 1 not detected')
        return 
    origin = ps[1]
    x = vo.sub(ps[2],ps[1])
    x = vo.div(x,vo.norm(x))
    y = vo.sub(ps[0],ps[1])
    y = vo.div(y,vo.norm(y))
    z = vo.cros(x,y)

    T_marker = (x+y+z,origin)
    return T_marker

def extractData1(save_path,cameras):
    """
    Extract data of marker1, the static marker that defines a transform
    
    Return:
    -----------------
    markers_l: A list of lists, where each list contains the detected transforms in the left camera
    qs_l: A list of lists, where each list contains the robot configurations at the pictures
    """

    markers_l = []
    qs_l = []
    markers_r = []
    qs_r = []
    for cn in cameras:
        for il,l in enumerate(open(save_path+'/robot_states.txt','r').readlines()):
            frame = cv2.imread(save_path + cn +"-%05d.jpg"%il)
        T_marker = detectMarker1(frame,cn)
        if T_marker is not None:
            line = [float(v) for v in l.split(' ') if l!='\n']
            if cn[10] == 'l':
                qs_l.append(line[0:6])
                markers_l.append(T_marker)
            elif cn[10] == 'r':
                qs_r.append(line[6:12])
                markers_r.append(T_marker)
            else:
                print('wrong camera name given')

    return markers_l,qs_l,markers_r,qs_r


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


def mainCalibration(traj_path,save_path,URDF_path,URDF_save_path,calibration_type,cameras,links,motion_address,codename):
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
        Tl,ql,Tr,qr = extractData1(save_path,cameras)
        URDFCalibration(Tl,ql,Tr,qr,URDF_path,URDF_save_path,links)

    elif calibration_type == 'moving':
        pass
    
    elif calibration_type == 'static':
        pass
