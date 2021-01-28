#Calibration functions for TRINA, without using Jarvis. Only dependent on motion and sensor modules.
#In this file, both the camera and URDF will be calibrated.
import os
# from klampt.model import trajectory as klamptTraj
from klampt.io import loader
from klampt import WorldModel
import sys
PATH = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if os.path.exists(PATH): 
    sys.path.remove(PATH)
    import cv2
    import cv2.aruco as aruco
    sys.path.append(PATH)
else:
    import cv2
    import cv2.aruco as aruco

# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from klampt.math import vectorops as vo
from klampt.math import so3
import trimesh
import numpy as np
from utils_my import *
import pickle


def pixelToP(x,y,pcd,dimy):
    tmp = pcd[y*dimy+x]
    return [tmp[0],tmp[1],tmp[2]]  ##data in folder 3/ only have y and z negated. In the future, should load just as is.
    # tmp = pcd[y]


def loadPcd(fn):
    data=open(fn,'r')
    columnPts=[]
    for line in data:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        columnPts.append(l2)
    data.close()

    return columnPts

def detectAruco(pic,marker_sz,IDs,dictionary,cn,use_depth = True,pcd = []):
    gray = cv2.cvtColor(pic, cv2.COLOR_BGR2GRAY)
    ##TODO: refactor this to put these in some settings file
    if cn == 'realsense_left':
        ##Instrinsics from factory settings
        # fx = 474.299
        # fy = 474.299
        # cx = 314.49
        # cy = 245.299
        # mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
        # dist = np.array([0.123036,0.135872,0.00483028,0.00670342,-0.0495168])

        ##Intrinsics from own calibration
        mtx = np.array([[608.88712495,   0.,         335.1847232 ],\
            [  0.,         613.83332425, 207.30088459],\
            [  0.,           0.,           1.,        ]])
        dist = np.array([ 6.15348981e-01,-4.62934706e+00,-9.15357406e-03,9.67396092e-03,1.09356865e+01])
        dimy = 640
    elif cn == 'realsense_right':
        ##Instrinsics from factory settings
        fx = 474.556
        fy = 474.556
        cx = 316.993
        cy = 245.439
        mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
        dist = np.array([0.156518,0.0823271,0.00524371,0.00575865,0.0232142])
        dimy = 640
    elif cn == 'zed_overhead':
        #1080p setting
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
        else: #4k settings
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
        dimy = 1920

    parameters = aruco.DetectorParameters_create()
    aruco_dict = aruco.Dictionary_get(dictionary)
    corners, detected_ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, \
                                                            cameraMatrix=mtx, distCoeff=dist)     
    #debugging
    # print(detected_ids)
    # aruco.drawDetectedMarkers(pic, [corners[0]])
    # # aruco.drawDetectedMarkers(pic, corners)
    # print(corners)
    # cv2.imshow('frame',pic)
    # cv2.waitKey(100000)
    # cv2.destroyAllWindows()

    #add the detected marker position 
    ps = []
    if detected_ids is not None:
        detected_ids = detected_ids.flatten().tolist()
        for target_id in IDs:
            if target_id in detected_ids:
                if use_depth:
                    corner = corners[detected_ids.index(target_id)]
                    p = [0]*3
                    missing_depth = False
                    for i in range(4):
                        corner_p = pixelToP(int(corner[0,i,0]),int(corner[0,i,1]),pcd,dimy = dimy)
                        if vo.norm(corner_p) < 1e-3: #depth value missing
                            missing_depth = True
                        p = vo.add(p,corner_p)
                    if missing_depth:
                        ps.append([])
                    else:
                        ps.append(vo.div(p,4.0))
                        print(target_id,vo.div(p,4.0))
                else:
                    #get the position
                    rvec,tvec,_=aruco.estimatePoseSingleMarkers(corners[detected_ids.index(target_id)],marker_sz,mtx,dist)
                    ps.append(tvec[0,0,:].tolist())
                    print(target_id,tvec[0,0,:])
            else:
                ps.append([])
    else:
        return []*len(IDs)

    return ps

def detectMarker1(pic,cn,use_depth = False,pcd = []):
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
    ps = detectAruco(pic,marker_sz,IDs,dictionary,cn,use_depth = True,pcd = pcd)
    if [] in ps or len(ps) < 3:
        print('marker 1 not detected')
        return 

    origin = ps[1]
    x = vo.sub(ps[0],ps[1])
    x = vo.div(x,vo.norm(x))
    y = vo.sub(ps[2],ps[1])
    y = vo.div(y,vo.norm(y))
    z = vo.cross(x,y)
    T_marker = (x+y+list(z),origin)

    return T_marker

def detectSingleMarker(pic,cn,use_depth = False,pcd = []):
    """
    Parameters:
    -------------------
    A picture containing marker2, which consists of a single Aruco marker of ID 

    Return:
    ------------------
    A 3d point in camera's frame
    """

    #TODO change these parameters
    IDs = [3]
    marker_sz = 0.04457
    dictionary = aruco.DICT_4X4_50
    ps = detectAruco(pic,marker_sz,IDs,dictionary,cn,use_depth = True,pcd = pcd)
    if [] in ps or len(ps) < 1:
        print('no marker detected')
        return 

    return ps[0]


def extractData1(save_path,cameras,use_depth = False):
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

            # if il < 50:
            #     continue

            print(save_path + cn +"-%05d.png"%il)
            frame = cv2.imread(save_path + cn +"-%05d.png"%il)
            if use_depth:
                pcd = loadPcd(save_path + cn +"-%05d.txt"%il)
                T_marker = detectMarker1(frame,cn,use_depth,pcd)
            else:
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


def extractData2(save_path,cameras,use_depth = False):
    """
    Extract data of a single aruco marker, its position 
    
    Return:
    -----------------
    markers: dict of markers extracted from different cameras
    qs: 
    """

    markers = {}
    qs = {}
    for cn in cameras:
        markers[cn] = []
        qs[cn] = []

    for cn in cameras:
        with open(save_path+'robot_state.txt','r') as f:
            lines = [line.rstrip() for line in f]
        for il,l in enumerate(lines):
            print(save_path + cn +"-%05d.png"%il)
            frame = cv2.imread(save_path + cn +"-%05d.png"%il)
            if use_depth:
                pcd = loadPcd(save_path + cn +"-%05d.txt"%il)
                p = detectSingleMarker(frame,cn,use_depth,pcd)
            else:
                p = detectSingleMarker(frame,cn)

            if p is not None:
                line = [float(v) for v in l.split(' ')]
                qs[cn].append(line[1:7]) #use left arm to attach the marker to
                markers[cn].append(p)

    return markers,qs


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
    

def mainCalibrationURDF(pic_path,robot_path,URDF_save_folder,cameras,links):
    """
    There are several different types of calibration ('URDF','wrist','static'). 
    1) URDF calibration, the arm mountings and 2 wrist mounted cameras will be calibrated. 
    The rest here assumes that the URDF has already been calibrated.
    2) Wrist mounted camera calibration. One or multiple cameras are calibrated. 
    3) Fixed camera calibration, where one or multiple fixed cameras will be calibrated.  

    calibration_type
    """
    folder = 'temp/'
    ## Process and save the extracted data
    # Tl,ql,Tr,qr = extractData1(pic_path,cameras,use_depth = True)
    # print('detected left limb camera markers:',len(Tl))
    # print('detected right limb camera markers:',len(Tr))
    
    # with open(folder + 'Tl', 'wb') as handle:
    #     pickle.dump(Tl, handle)
    # with open(folder + 'ql', 'wb') as handle:
    #     pickle.dump(ql, handle)
    # with open(folder + 'Tr', 'wb') as handle:
    #     pickle.dump(Tr, handle)
    # with open(folder + 'qr', 'wb') as handle:
    #     pickle.dump(qr, handle)
  

    #load the extracted data
    with open(folder + 'Tl', 'rb') as handle:
        Tl = pickle.load(handle)
    with open(folder + 'ql', 'rb') as handle:
        ql = pickle.load(handle)
    with open(folder + 'Tr', 'rb') as handle:
        Tr = pickle.load(handle)
    with open(folder + 'qr', 'rb') as handle:
        qr = pickle.load(handle)

    from calibrationOpt import URDFCalibration
    import math
    T_c_l_0 = ([math.sqrt(2)/2,0,-math.sqrt(2)/2,-math.sqrt(2)/2,0,-math.sqrt(2)/2,0,1,0],[0.05,-0.05,0.07])
    T_c_r_0 = ([1,0,0,0,0,-1,0,1,0],[0.0,-0.05,0.1])
    T_marker_1 = ([1,0,0,0,1,0,0,0,1],[0.15,0.0,0.363]) #This is exact
    T_base_l, T_c_l,T_base_r, T_c_r = URDFCalibration(Tl,ql,Tr,qr,T_marker_1,T_c_l_0,T_c_r_0,robot_path,links)
    from klampt.math import so3
    print('left base rpy,t:',so3.rpy(T_base_l[0]),T_base_l[1])
    print('left camera transform:',T_c_l)
    print('left camera R row major:',col_to_row_major(T_c_l[0]))
    # print('right base rpy,t:',so3.rpy(T_base_r[0]),T_base_r[1])



    # #now modify the URDF 
    # Hardcode these for now. Set these in the setting file in the future
    world = WorldModel()
    res = world.loadElement(robot_path)
    robot = world.robot(0)

    robot.link(10).setParentTransform(T_base_l[0],T_base_l[1])
    # robot.link(18).setParentTransform(T_base_r[0],T_base_r[1])
    loader.save(robot,'auto',URDF_save_folder + 'Cholera_calibrated.rob') #klampt loader only saves a robot to .rob. In addition, it does not save the meshes, and will keep
    
    #modify the mesh
    camera_l = trimesh.load_mesh('~/TRINA/Motion/data/robots/sensors/SR305.STL')
    # camera_r = trimesh.load_mesh('~/TRINA/Motion/data/robots/sensors/SR305.STL')
    T_c_l = klampt_to_np(T_c_l)
    # T_c_r = klampt_to_np(T_c_r)

    from numpy.linalg import inv
    camera_l.apply_transform(T_c_l)
    # camera_r.apply_transform(T_c_r)
    l_link5 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Cholera/left_wrist2_link.STL')
    # r_link5 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Cholera/right_wrist2_link.STL')
    l_link5_new = trimesh.util.concatenate([l_link5,camera_l])
    # r_link5_new = trimesh.util.concatenate([r_link5,camera_r])
    # ## The camera is merged with the wrist 2 geometry
    l_link5_new.export('~/TRINA/Motion/data/robots/Cholera_calibrated/left_wrist2_link.STL')
    # r_link5_new.export('~/TRINA/Motion/data/robots/Cholera_calibrated/right_wrist2_link.STL')

    # lower/raise the base link
    # TODO: break the base link into 2 links to accomodate calibration
    # base_link = trimesh.load_mesh('~/TRINA/Motion/data/robots/Cholera/base_link.STL')
    # print(zl,zr,T_base_l[1][2],T_base_r[1][2])
    # print([0,0,(zl+zr-T_base_l[1][2]-T_base_r[1][2])/2])
    # print(yl,yr,T_base_l[1][1],T_base_r[1][1])
    # base_link.apply_transform(klampt_to_np(([1,0,0,0,1,0,0,0,1],[0,0,(-zl-zr+T_base_l[1][2]+T_base_r[1][2])/2])))
    # base_link.export('~/TRINA/Motion/data/robots/Cholera_calibrated/base_link.STL')

    
    # ####Final step
    # ##need to change the mesh name in the generated new .rob file to the correct mesh files
    # ##new to edit the sensors.xml files to the right link and transforms


# def mainCalibrationStatic(traj_path,pic_path,world_path,cameras,link):
#     ps,qs = extractData2(pic_path,cameras,use_depth = True)
#     print('detected realsense torso camera markers:',len(ps[cameras[0]]))
#     print('detected zed torso camera markers:',len(ps[cameras[1]]))
#     folder = 'temp/'
#     with open(folder + 'ps', 'wb') as handle:
#         pickle.dump(ps, handle)
#     with open(folder + 'qs', 'wb') as handle:
#         pickle.dump(qs, handle)

#     # with open(folder + 'ps', 'rb') as handle:
#     #     ps = pickle.load(handle)
#     # with open(folder + 'qs', 'rb') as handle:
#     #     qs = pickle.load(handle)

#     T_realsense_0 = ([1,0,0,0,0,-1,0,1,0],[0.0,-0.05,0.1])
#     T_zed_0 = ([1,0,0,0,1,0,0,0,1],[0,0,0])
#     T_marker = ([1,0,0,0,1,0,0,0,1],[0,0,0])
#     T_0 = [T_realsense_0,T_zed_0]
#     T_calibrated = StaticCalibration(T_0,T_zedql,Tr,qr,T_marker_1,T_c_l_0,T_c_r_0,world_path,URDF_save_folder + 'Bubonic.urdf',link)

#     ##now modify the URDF 
#     ##Hardcode these for now. Set these in the setting file in the future
#     world = WorldModel()
#     res = world.loadElement(robot_path)
#     robot_model = world.robot(0)

#     robot.link(10).setParentTransform(T_base_l[0],T_base_l[1])
#     robot.link(18).setParentTransform(T_base_r[0],T_base_r[1])
#     loader.save(robot,'auto',URDF_save_folder + 'cholera_calibrated.rob') #klampt loader only saves a robot to .rob. In addition, it does not save the meshes, and will keep
    
#     # #modify the mesh
#     camera_l = trimesh.load_mesh('~/TRINA/Motion/data/robots/sensors/SR305.STL')
#     camera_r = trimesh.load_mesh('~/TRINA/Motion/data/robots/sensors/SR305.STL')
#     T_c_l = klampt_to_np(T_c_l)
#     T_c_r = klampt_to_np(T_c_r)

#     from numpy.linalg import inv
#     camera_l.apply_transform(T_c_l)
#     camera_r.apply_transform(T_c_r)
#     l_link5 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Anthrax_lowpoly/left_wrist2_link.STL')
#     r_link5 = trimesh.load_mesh('~/TRINA/Motion/data/robots/Anthrax_lowpoly/right_wrist2_link.STL')
#     l_link5_new = trimesh.util.concatenate([l_link5,camera_l])
#     r_link5_new = trimesh.util.concatenate([r_link5,camera_r])
#     # ## The camera is merged with the wrist 2 geometry
#     l_link5_new.export('~/TRINA/Motion/data/robots/Bubonic_calibrated/left_wrist2_link.STL')
#     r_link5_new.export('~/TRINA/Motion/data/robots/Bubonic_calibrated/right_wrist2_link.STL')

#     ####Final step
#     ##need to change the mesh name in the generated new .rob file to the correct mesh files
#     ##new to edit the sensors.xml files to the right link and transforms

if __name__=="__main__":

    ####################
    # URDF calibration #
    ####################

    traj_path = 'URDF_calibration.path'
    pic_path = './data/0127Test/'
    robot_path = '../../Motion/data/robots/Cholera.urdf'
    rob_save_folder = '../../Motion/data/robots/'
    cameras =['realsense_left']#,'realsense_right']

    ##Take pictures
    # from calibrationLogger import CalibrationLogger
    # takePictures(traj_path,pic_path,cameras,'http://localhost:8080','cholera')
    ##Calibrate
    mainCalibrationURDF(pic_path = pic_path, robot_path = robot_path,URDF_save_folder = rob_save_folder,\
        cameras = cameras,links = [15,23])

    #############################
    # static camera calibration #
    #############################

    # ### Static camera calibration
    # traj_path = 'static_calibration.path'
    # pic_path = './data/5/'
    # world_path = '../../Motion/data/TRINA_world_cholera.xml'
    # cameras = ['realsense_left','realsense_right'] #['realsense_torso','zed_torso'] 
    # ##Take pictures
    # from calibrationLogger import CalibrationLogger
    # takePictures(traj_path,pic_path,cameras = cameras,motion_address = 'http://localhost:8080',codename = 'cholera')
    # # mainCalibrationStatic(traj_path = traj_path,pic_path = pic_path, world_path = world_path,cameras = cameras,link = 11)