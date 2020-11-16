#Calibration functions for TRINA, without using Jarvis. Only dependent on motion and sensor modules.
#In this file, both the camera and URDF will be calibrated.
import os
# from klampt.model import trajectory as klamptTraj
from klampt.io import loader
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import cv2.aruco as aruco
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from klampt.math import vectorops as vo
import trimesh
import numpy as np
from calibrationOpt import URDFCalibration

def detectAruco(pic,marker_sz,IDs,dictionary,cn):
    gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
    if cn == 'realsense_left':
        fx = 474.299
        fy = 474.299
        cx = 314.49
        cy = 245.299
        mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
        dist = np.array([0.123036,0.135872,0.00483028,0.00670342,-0.0495168])
    elif cn == 'realsense_right':
        fx = 474.556
        fy = 474.556
        cx = 316.993
        cy = 245.439
        mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
        dist = np.array([0.156518,0.0823271,0.00524371,0.00575865,0.0232142])
    elif cn == 'zed_overhead':
        if pic.shape==(1920,1080):
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
    aruco_dict = aruco.Dictionary_get(dictionary)
    corners, detected_ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, \
                                                            cameraMatrix=mtx, distCoeff=dist)     
    #debugging
    aruco.drawDetectedMarkers(pic, corners)
    cv2.imshow('frame',pic)
    cv2.waitKey(100000)
    cv2.destroyAllWindows()



    #add the detected marker position 
    ps = []
    if detected_ids is not None:
        detected_ids = detected_ids.flatten().tolist()
        for target_id in IDs:
            if target_id in detected_ids:
                #get the position
                rvec,tvec,_=aruco.estimatePoseSingleMarkers(corners[detected_ids.index(target_id)],marker_sz,mtx,dist)
                ps.append(tvec[0,0,:].tolist())

                print(target_id,tvec[0,0,:])
            else:
                ps.append([])
    else:
        return [[],[],[]]
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
    IDs = [0,1,2]
    marker_sz = 0.06
    dictionary = aruco.DICT_4X4_50
    ps = detectAruco(pic,marker_sz,IDs,dictionary,cn)
    if [] in ps:
        print('marker 1 not detected')
        return 

    origin = ps[1]
    x = vo.sub(ps[0],ps[1])
    x = vo.div(x,vo.norm(x))
    y = vo.sub(ps[2],ps[1])
    y = vo.div(y,vo.norm(y))
    z = vo.cross(x,y)
    T_marker = (x+y+list(z),origin)

    # exit()
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
        with open(save_path+'robot_state.txt','r') as f:
            lines = [line.rstrip() for line in f]
        for il,l in enumerate(lines):
            print(save_path + cn +"-%05d.png"%il)
            frame = cv2.imread(save_path + cn +"-%05d.png"%il)
            T_marker = detectMarker1(frame,cn)
            if T_marker is not None:
                line = [float(v) for v in l.split(' ')]
                if cn[10] == 'l':
                    qs_l.append(line[1:7])
                    markers_l.append(T_marker)
                elif cn[10] == 'r':
                    qs_r.append(line[7:13])
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

def klampt_to_np(T):
    (R,t) = T
    T = np.array([[R[0],R[3],R[6],t[0]],\
                  [R[1],R[4],R[7],t[1]],\
                  [R[2],R[5],R[8],t[2]],\
                  [0,0,0,1]])
    return T
    

def mainCalibration(traj_path,save_path,world_path,URDF_save_folder,calibration_type,cameras,links,motion_address,codename):
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
        # from calibrationLogger import CalibrationLogger
        # takePictures(traj_path,save_path,cameras,motion_address,codename)
        Tl,ql,Tr,qr = extractData1(save_path,cameras)
        print('detected left limb camera markers:',len(Tl))
        print('detected left limb camera markers:',len(Tr))


        print(Tl[0])
        # np.save('Tl.npy',np.array(Tl))
        # np.save('ql.npy',np.array(ql))
        # np.save('Tr.npy',np.array(Tr))
        # np.save('qr.npy',np.array(qr))

        # Tl = np.load('Tl.npy').tolist()
        # ql = np.load('ql.npy').tolist()
        # Tr = np.load('Tr.npy').tolist()
        # qr = np.load('qr.npy').tolist()


        # T_c_l_0 = ([1,0,0,0,0,-1,0,1,0],[0.0,-0.05,0.1])
        # T_c_r_0 = ([1,0,0,0,1,0,0,0,1],[0,0,0])
        # T_marker_1 = ([1,0,0,0,1,0,0,0,1],[0.15,0.0,0.363]) #This is exact
        # T_base_l, T_c_l,T_base_r, T_c_r = URDFCalibration(Tl,ql,Tr,qr,T_marker_1,T_c_l_0,T_c_r_0,world_path,URDF_save_folder + 'Bubonic.urdf',links)

        # #now modify the URDF 
        # robot = loader.load(URDF_path)
        # robot.link(6).setParentTransform(T_base_l[0],T_base_l[1])
        # robot.link(14).setParentTransform(T_base_r[0],T_base_r[1])
        # load.save(robot,'auto',URDF_save_path) #klampt loader only saves a robot to .rob. In addition, it does not save the meshes, and will keep
        # #the same mesh reference path. 
        # # mesh1 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Anthrax_lowpoly/left_wrist2_link.STL')
        # # mesh2 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Anthrax_lowpoly/SR305.STL')
        # #modify the mesh
        # camera_l = trimesh.load_mesh('~/TRINA/Motion/data/robots/sensors/SR305.STL')
        # camera_r = trimesh.load_mesh('~/TRINA/Motion/data/robots/sensors/SR305.STL')
        # T_c_l = klampt_to_np(T_c_l)
        # T_c_r = klampt_to_np(T_c_l)

        # T_c_link5 = np.array([[0,0,-1,0],\
        #                       [0,1,0,0],\
        #                       [-1,0,0,0],\
        #                       [0,0,0,1]]) 
        # from numpy.linalg import inv
        # camera_l.apply_transform(inv(T_c_link5)*T_c_l*T_c_link5)
        # camera_r.apply_transform(inv(T_c_link5)*T_c_r*T_c_link5)
        # l_link5 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Anthrax_lowpoly/left_wrist2_link.STL')
        # r_link5 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Anthrax_lowpoly/right_wrist2_link.STL')
        # l_link5_new = trimesh.util.concatenate([l_link5,camera_l])
        # r_link5_new = trimesh.util.concatenate([r_link5,camera_r])
        # ## The camera is merged with the wrist 2 geometry
        # l_link5_new.save_mesh('~/TRINA/Motion/data/robots/Bubonic_calibrated/left_wrist2_link.STL')
        # r_link5_new.save_mesh('~/TRINA/Motion/data/robots/Bubonic_calibrated/right_wrist2_link.STL')

        # ##need to change the mesh name in the generated new .rob file to the correct 
    elif calibration_type == 'moving': 
        pass
    
    elif calibration_type == 'static':
        pass


if __name__=="__main__":
    traj_path = ' '
    save_path = './data/1/'
    world_path = '../../Motion/data/TRINA_world_bubonic_calibration.xml'
    rob_save_folder = '../../Motion/data/robots/'

    mainCalibration(traj_path = traj_path,save_path = save_path, world_path = world_path,URDF_save_folder = rob_save_folder,\
        calibration_type = 'URDF',cameras=['realsense_left','realsense_right'],links = [11,19],motion_address = 'http://localhost:8080',\
        codename = 'bubonic')