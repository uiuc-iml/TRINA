import sys
sys.path.append('../../')
from SensorModule import Camera_Robot
sys.path.append('../../Motion/')
import TRINAConfig

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import cv2.aruco as aruco
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
from klampt.math import se3
import time
from copy import copy
sensor_module = Camera_Robot(robot = [],world = [], cameras =['zed_overhead'],ros_active = False, use_jarvis = False, mode = 'Physical')

res = sensor_module.get_rgbd_images()
color_frame = res['zed_overhead'][0]
frame = np.asarray(color_frame)
sensor_module.safely_close_all()

frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
#left 1080
# fx=1055.28
# fy=1054.69
# cx=1126.61
# cy=636.138
# k1=-0.0426844
# k2=0.0117943
# k3=-0.00548354
# p1=0.000242741
# p2=-0.000475926

#2K
fx=1055.28
fy=1054.69
cx=1126.61
cy=636.138
k1=-0.0426844
k2=0.0117943
k3=-0.00548354
p1=0.000242741
p2=-0.000475926
matrix_coefficients = np.array([[fx,0,cx],\
    [0,fy,cy],\
    [0,0,1]])

distortion_coefficients = np.array([k1,k2,p1,p2,k3])
distortion_coefficients = distortion_coefficients.flatten()
# matrix_coefficients = np.array([[1.03453134e+03, 0.00000000e+00, 1.10499530e+03],
#             [0.00000000e+00, 1.03700020e+03, 6.39287393e+02],
#             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
# distortion_coefficients  = np.array([-1.58720973e+00,3.03474963e+00,-5.39350262e-04,-1.90796795e-03,-2.66601914e+00,-1.55093063e+00,2.96539748e+00,-2.63263175e+00])



gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

# cv2.imshow('gray',gray)
# cv2.waitKey(1000000)
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)  # Use 5x5 dictionary to find markers
# aruco_dict = aruco.DICT_7X7_50
# aruco_dict = aruco.Dictionary_get(aruco_dict)
parameters = aruco.DetectorParameters_create()  # Marker detection parameters
# lists of ids and the corners beloning to each id
corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
    parameters=parameters)
    # cameraMatrix=matrix_coefficients,
    # distCoeff=distortion_coefficients)

ps = []

if np.all(ids is not None):  # If there are markers found by detector
    for i in range(len(ids)):  # Iterate in markers

        # if ids[i][0] == 3:
        # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
        # if ids[i][0] == 1:
        #     size = 0.0355
        # elif ids[i][0] == 5:
        #     size = 0.0515
        size = 0.0378
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i],size, matrix_coefficients,
                                                                    distortion_coefficients)
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
        aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
        # cv2.imshow('frame',frame)
        # cv2.waitKey(1000000)
        marker_p = tvec[0][0]
        print(ids[i][0])
        print('p in camera:',marker_p)
        # if ids[i][0] == 5:
        #     ps.append(marker_p)

        # if ids[i][0] == 1:
        Tc = TRINAConfig.fixed_camera_transform
        marker_p_world = se3.apply(Tc,marker_p)
        print('p in world:',marker_p_world)

        # if marker_p_world[2] > 0.8:
        sys.path.append('../../Motion/')
        from motion_client_python3 import MotionClient
        #At this point the calibration data has not been loaded into Motion yet
        server_address = 'http://10.0.242.158:8080' 
        server_address = server_address
        robot =  MotionClient(address = server_address)


        robot.startServer(mode = 'Physical',components = ['right_limb'],codename = 'bubonic')
        robot.startup()
        time.sleep(1)

        # print(robot.sensedRightEETransform())
        #right limb
        # T_g = ([-0.011249909559866876, 0.04027310229041789, -0.9991253758997424, -0.012546404743281269, 0.9991042663598395, 0.04041352087590054, 0.9998580035499612, 0.012990079810171208, -0.010734549995821463],[0,0,0])
        
        #left limb
        T_g = ([-0.08522872297083785, 0.06394837732163343, -0.9943071305279326, -0.9852487806077552, 0.1432201348831597, 0.09366340413941264, 0.14839442405863318, 0.9876227002261438, 0.050798591578180424],[0,0,0])
    

        # marker_p_world = [0.5748583080916612, 0, 0.7532123108420521]
        T_g[1][0] = float(marker_p_world[0])
        T_g[1][1] = float(marker_p_world[1])
        T_g[1][2] = float(marker_p_world[2]) + 0.25

        robot.setRightEEInertialTransform(T_g,15)
        time.sleep(16)
        robot.shutdown()
        break


else:
    print('no marker detected')


# from klampt.math import vectorops
# for i in range(3):
#     print(vectorops.norm(vectorops.sub(ps[i],ps[i+1])))