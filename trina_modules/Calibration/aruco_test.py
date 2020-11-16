import time,sys
sys.path.append('../../')
from SensorModule import Camera_Robot
sys.path.append('../../Motion/')
import cv2,cv2.aruco as aruco
import numpy as np

#cn = 'realsense_left'
cn = 'zed_overhead'
camera = Camera_Robot(robot = [],world = [], cameras = [cn],ros_active = False, use_jarvis = False, mode = 'Physical')
time.sleep(2)
res = camera.get_rgbd_images()
color_frame = res[cn][0]
# color = cv2.cvtColor(np.asarray(color_frame), cv2.COLOR_RGB2BGR)
color = np.asarray(color_frame)
# color = color[:,:,0:3]
cv2.imwrite('color.png',color)

color = cv2.imread('color.png')
gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
pic = color
print(pic.shape)
if pic.shape==(1920,1080,3):
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



# fx = 474.299
# fy = 474.299
# cx = 314.49
# cy = 245.299
# mtx = np.array([[fx,0,cx], [0,fy,cy], [0,0,1]])
# dist = np.array([0.123036,0.135872,0.00483028,0.00670342,-0.0495168])

parameters = aruco.DetectorParameters_create()
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
corners, detected_ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters, \
                                                        cameraMatrix=mtx, distCoeff=dist)    



#debugging
aruco.drawDetectedMarkers(color, corners)
cv2.imshow('frame',color)
cv2.waitKey(100000)
cv2.destroyAllWindows()

camera.safely_close_all()

 #add the detected marker position 
ps = []
IDs = [0,1,2,3,4]
print(detected_ids)
if detected_ids is not None:
    detected_ids = detected_ids.flatten().tolist()
    for target_id in IDs:
        if target_id in detected_ids:
            #get the position
            if target_id  < 3:
                marker_sz = 0.06
            else:
                marker_sz = 0.045
            rvec,tvec,_=aruco.estimatePoseSingleMarkers(corners[detected_ids.index(target_id)],marker_sz,mtx,dist)
            ps.append(tvec[0,0,:].tolist())

            print(target_id,tvec[0,0,:])
        else:
            ps.append([])