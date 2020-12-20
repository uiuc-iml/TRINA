import sys
import numpy as np
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

pic = cv2.imread('../../../Downloads/data_0/depth_wirst-00001.png', cv2.IMREAD_UNCHANGED)

print(pic)
# (m,n,l) = np.shape(pic)
# print('shape:',np.shape(pic))

# for i in range(m):
#     for j in range(n):
#         if pic[i,j] > 0:
#             print(pic[i,j])

# import time
# import sys
# sys.path.append('../../Motion/')
# from motion_client import MotionClient
# server_address = 'http://10.0.242.158:8080' 
# server_address = server_address
# robot =  MotionClient(address = server_address)


# print(robot.sensedLeftEETransform())