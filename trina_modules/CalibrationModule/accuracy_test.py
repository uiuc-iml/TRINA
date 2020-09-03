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
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

# fx=1055.18
# fy=1054.3
# cx=965.66
# cy=554.859
# k1=-0.0400314
# k2=0.00840801
# k3=-0.00439891
# p1=0.000469178
# p2=-7.16881e-05

fx=1055.28
fy=1054.69
cx=982.61
cy=555.138
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
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

# cv2.imshow('gray',gray)
# cv2.waitKey(1000000)
aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)  # Use 5x5 dictionary to find markers
parameters = aruco.DetectorParameters_create()  # Marker detection parameters
# lists of ids and the corners beloning to each id
corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
    parameters=parameters)
    # cameraMatrix=matrix_coefficients,
    # distCoeff=distortion_coefficients)
if np.all(ids is not None):  # If there are markers found by detector
    for i in range(0, len(ids)):  # Iterate in markers
        # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.038, matrix_coefficients,
                                                                    distortion_coefficients)
        (rvec - tvec).any()  # get rid of that nasty numpy value array error
        aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
        aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
        
        marker_p = tvec[0][0]
        Tc = TRINAConfig.fixed_camera_transform
        marker_p_world = se3.apply(Tc,marker_p)

        print(id,marker_p_world)
cv2.imshow('frame', frame)
# Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
key = cv2.waitKey(300000000)


# if ids is not None:
#     aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
#     aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
#     marker_p = tvec[0][0]

#     print('p in camera:',marker_p)
#     Tc = TRINAConfig.fixed_camera_transform
#     marker_p_world = se3.apply(Tc,marker_p)
#     print('p in world:',marker_p_world)




#     # sys.path.append('../../Motion/')
#     # from motion_client_python3 import MotionClient
#     # #At this point the calibration data has not been loaded into Motion yet
#     # server_address = 'http://localhost:8080' 
#     # server_address = server_address
#     # robot =  MotionClient(address = server_address)


#     # robot.startServer(mode = 'Physical',components = ['right_limb'],codename = 'bubonic')
#     # robot.startup()
#     # time.sleep(1)

#     # # print(robot.sensedRightEETransform())
#     # T_g = ([-0.020720845975319327, -0.007470379074614328, -0.9997573905596049, -0.6632461464423366, -0.7481514831493108, 0.019336687691494693, \
#     #     -0.7481144269237562, 0.663485909193232, 0.01054763151484276],[0,0,0])
#     # T_g[1][0] = float(marker_p_world[0])
#     # T_g[1][1] = float(marker_p_world[1])
#     # T_g[1][2] = float(marker_p_world[2]) + 0.25

#     # robot.setRightEEInertialTransform(T_g,10)
#     # time.sleep(12)
#     # robot.shutdown()


# else:
#     print('no marker detected')