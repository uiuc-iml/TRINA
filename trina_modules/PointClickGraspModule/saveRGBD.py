import sys
sys.path.append('../../')
from SensorModule import Camera_Robot
sys.path.append('../../Motion/')
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
#sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import numpy as np
import open3d as o3d
def readImage(path): 

    pcd = o3d.io.read_point_cloud(path)
    o3d.visualization.draw_geometries([pcd])
    return

class RGBDSaver:
    def __init__(self,cameras = ['realsense_right']):
        self.camera = Camera_Robot(robot = [],world = [], cameras =['realsense_right'],ros_active = False, use_jarvis = False, mode = 'Physical')
        
    def saveRGBD(self,path,index,camera):
        res = self.camera.get_rgbd_images()
        color_frame = res[camera][0]
        depth_frame = res[camera][1]

        color = cv2.cvtColor(np.asarray(color_frame), cv2.COLOR_RGB2BGR)
        depth = np.asarray(depth_frame)

        format = '.png'

        res = self.camera.get_point_clouds()
        pcd = res[camera]
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # print(np.asarray(pcd.points))
        o3d.io.write_point_cloud(path + '-' + index + '.pcd',pcd)
        cv2.imwrite(path + '-color-' + index + format,color) #,[int(cv2.IMWRITE_JPEG_QUALITY), 99])
        cv2.imwrite(path + '-depth-' + index + format,depth)
        
        return
    
if __name__=="__main__":
    #readImage('item_set/item1-41.pcd')
    parser = ArgumentParser(description='logger', formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument('path',type = str)
    parser.add_argument('camera',type = str)
    args = parser.parse_args()

    saver = RGBDSaver()
    waitindex = input("Input a number for naming the file\n")
    while waitindex != "q":
    
        saver.saveRGBD(args.path, waitindex, args.camera)
        waitindex = input("Input a number for naming the file\n")
