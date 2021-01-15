import time,sys
sys.path.append('../../')
from SensorModule import Camera_Robot
sys.path.append('../../Motion/')
import cv2,cv2.aruco as aruco
import numpy as np
import open3d as o3d
cn = 'realsense_spare'
# cn = 'zed_overhead'
camera = Camera_Robot(robot = [],world = [], cameras = [cn],ros_active = False, use_jarvis = False, mode = 'Physical')
time.sleep(2)

# for i in range(100):
#     res = camera.get_rgbd_images()
#     color_frame = res[cn][0]
#     color = cv2.cvtColor(np.asarray(color_frame), cv2.COLOR_RGB2BGR)
#     color = np.asarray(color_frame)
#     cv2.imwrite('cal_pics/'+str(i+200)+'.png',color)

#     cv2.imshow('frame',color)
#     cv2.waitKey(100000)
#     cv2.destroyAllWindows()

res = camera.get_point_clouds()
pcd = res[cn]

o3d.io.write_point_cloud("yifan1.pcd", pcd)

camera.safely_close_all()

o3d.visualization.draw_geometries([pcd])
