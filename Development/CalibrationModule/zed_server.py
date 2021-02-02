""" this file servers both as a camera server and a logger"""

from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import xmlrpc.client
import sys
import signal
sys.path.append('../../')
from SensorModule import Camera_Robot
sys.path.append('../../Motion/')
import TRINAConfig
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from threading import Thread, Lock, RLock
import threading
import time
import numpy as np
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from motion_client import MotionClient
class CameraServerLogger:
    def __init__(self,ip_address = 'localhost',port = 8040, path= '.',robot_address ='http://10.0.242.158:8080',save = False):
        self.camera = Camera_Robot(robot = [],world = [], cameras =['zed_overhead'],ros_active = False, use_jarvis = False, mode = 'Physical')
        self.robot =  MotionClient(address = robot_address)
        self.robot.startServer(mode = 'Physical',components = ['left_limb','right_limb'],codename = 'bubonic')
        self.robot.startup()
        self.ip_address = ip_address
        self.port = port
        self.shutdown_flag = False
        self.pics = []
        self.camera_dt = 0.2
        self.state_dt = 0.01
        self.system_start_time = time.time()
        self.path = path
        self._lock = RLock()
        self.save = save
        cameraLoggingThread = threading.Thread(target = self._cameraLoggingLoop)
        cameraLoggingThread.start()
        robotLoggingThread = threading.Thread(target = self._robotLoggingLoop)
        robotLoggingThread.start()
        serverThread = threading.Thread(target = self._serverLoop)
        serverThread.start()

   
    def _cameraLoggingLoop(self):
        """
        The depth channel uses uint16, i.e. 0-65535
        The maximum depth of realsense is 1.5m
        
        Note that the depth frame returned by realsense is already in depth image format.
        The depth frame from zed is 2D numpy array of floats. Need to be converted.

        Arbitrarily set the max depth of zed to be 5
        """
        counter = -1
        if self.save:
            f = open(self.path + 'camera_stamps.txt','w')
        format = '.jpg'
        while not self.shutdown_flag:
            loop_start_time = time.time()
            self._lock.acquire()
            self.res = self.camera.get_rgbd_images()
            self._lock.release()

            overhead_color_frame = self.res['zed_overhead'][0]
            overhead_depth_frame = self.res['zed_overhead'][1]
            overhead_color = np.asarray(overhead_color_frame)
            overhead_depth = np.asarray(overhead_depth_frame)
            # left_color_frame = self.res['realsense_left'][0]
            # left_depth_frame = self.res['realsense_left'][1]
            # left_color = cv2.cvtColor(np.asarray(left_color_frame), cv2.COLOR_RGB2BGR)
            # left_depth = np.asarray(left_depth_frame)
            # right_color_frame = self.res['realsense_right'][0]
            # right_depth_frame = self.res['realsense_right'][1]
            # right_color = cv2.cvtColor(np.asarray(right_color_frame), cv2.COLOR_RGB2BGR)
            # right_depth = np.asarray(left_depth_frame)   

            if self.save:
                #process the depth returned by zed
                where_are_NaNs = np.isnan(overhead_depth)
                overhead_depth[where_are_NaNs] = 0
                overhead_depth[overhead_depth > 5.0] = 0
                overhead_depth = overhead_depth/5.0*65536.0
                overhead_depth = overhead_depth.astype(int)
                #the first frame takes a while
                if counter >= 0:
                    cv2.imwrite(self.path + 'color_overhead-' + str(counter).zfill(5)+format,overhead_color,[int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    # cv2.imwrite(self.path + 'color_left-' + str(counter).zfill(5)+format,left_color,[int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    # cv2.imwrite(self.path + 'color_right-' + str(counter).zfill(5)+format,right_color,[int(cv2.IMWRITE_JPEG_QUALITY), 90])

                    # cv2.imwrite(self.path + 'depth_right-' + str(counter).zfill(5)+'.png',right_depth)
                    # cv2.imwrite(self.path + 'depth_left-' + str(counter).zfill(5)+'.png',left_depth)
                    cv2.imwrite(self.path + 'depth_overhead-' + str(counter).zfill(5)+'.png',overhead_depth)
                    elapsed_time = time.time() - loop_start_time
                    if elapsed_time < self.camera_dt:
                        time.sleep(self.camera_dt - elapsed_time)
                    else:
                        time.sleep(0.00001)
                    f.write(str(time.time() - self.system_start_time))
                    f.write('\n')
                    print(elapsed_time)
                counter += 1
                    
            # break
        if self.save:   
            f.close()

    def _robotLoggingLoop(self):
        if self.save:
            with open(self.path+'robot_state.txt','w') as f:
                while not self.shutdown_flag:
                    if self.robot.isShutDown():
                        self.shutdown()
                        break
                    t = time.time() - self.system_start_time
                    f.write(str(t)+' ')
                    q = self.robot.sensedLeftLimbPosition()
                    for ele in q:
                        f.write(str(ele)+' ')
                    q = self.robot.sensedRightLimbPosition()
                    for ele in q:
                        f.write(str(ele)+' ')
                    wrench = self.robot.sensedLeftEEWrench()
                    for ele in wrench:
                        f.write(str(ele)+' ')
                    wrench = self.robot.sensedRightEEWrench()
                    for ele in wrench:
                        f.write(str(ele)+' ')
                    f.write('\n')
                    time.sleep(self.state_dt)

                    

    def _serverLoop(self):

        def _getCameraTransform():
            # return TRINAConfig.fixed_camera_transform
            return self.c

        def _getPicture():
            return str(self.res['zed_overhead'][0].tolist())

        def _getURDF(vss,vis):
            initial_urdf_path = '../../Motion/data/robots/Bubonic_uncalibrated.urdf'
            final_urdf_path = '../../../Downloads/tmp.urdf'
            from dense_calibration import extrinsic_calibration_read
            import glob
            root_path = '/data/test_data/'
            folder_list = glob.glob(root_path + 'calibration_*')
            if len(folder_list)>0:
                number_list = []
                for folder in folder_list:
                    number = [int(s) for s in folder.split('_') if s.isdigit()]
                    number_list.append(number[0])
                    largest_number = np.max(number_list)
                    path = root_path + 'calibration_' + str(largest_number)
            else:
                print('no folder detected')
                exit()
            c =    \
            extrinsic_calibration_read( folder= path,lPos=None,rPos=None,
                                        ROBOT_PATH= initial_urdf_path,
                                        ROBOT_PATH_OUT=final_urdf_path,
                                        desired_base_mesh=vss,
                                        vis=vis)
            
            self.c = (np.array(c[0]).astype(float).tolist(),np.array(c[1]).astype(float).tolist())

            with open(final_urdf_path, "rb") as handle:
                return xmlrpc.client.Binary(handle.read())



        self.server = SimpleXMLRPCServer((self.ip_address,self.port), logRequests=False)
        self.server.register_introspection_functions()
        self.server.register_function(_getCameraTransform,'getCameraTransform')
        self.server.register_function(_getPicture,'getPicture')
        self.server.register_function(_getURDF,'getURDF')
        print('Server Created')
        self.server.serve_forever()


    def shutdown(self):
        self._lock.acquire()
        self.shutdown_flag = True
        #self.camera.safely_close_all()
        self.server.shutdown()
        print("shutdown called")
        self._lock.release()

if __name__=="__main__":
    parser = ArgumentParser(description='Zed server and logger', formatter_class=ArgumentDefaultsHelpFormatter)
    # parser.add_argument('No', type=str, help='test number')
    parser.add_argument('type',type = str, help ='calibration or regular logging')
    parser.add_argument('save',type = int)
    args = parser.parse_args()
    #'10.0.242.158''localhost'

    #check existing directories
    import glob
    import os
    root_path = '/data/test_data/'
    if args.type == 'calibration':
        folder_list = glob.glob(root_path + 'calibration_*')
        if len(folder_list)>0:
            number_list = []
            for folder in folder_list:
                number = [int(s) for s in folder.split('_') if s.isdigit()]
                number_list.append(number[0])
            largest_number = np.max(number_list)
            path = root_path + 'calibration_' + str(largest_number + 1) +'/'
        else:
            path = root_path + 'calibration_0/'

    elif args.type == 'logging':
        folder_list = glob.glob(root_path + 'logging_*')
        if len(folder_list)>0:
            number_list = []
            for folder in folder_list:
                number = [int(s) for s in folder.split('_') if s.isdigit()]
                number_list.append(number[0])
            largest_number = np.max(number_list)
            path = root_path + 'logging_' + str(largest_number + 1) + '/'
        else:
            path = root_path + 'logging_0/'


    os.mkdir(path)
    server = CameraServerLogger(ip_address = '10.0.242.158',port = 8040, path = path ,save = args.save)
    def sigint_handler(signum, frame):
        """ Catch Ctrl+C tp shutdown 
        """
        assert(signum == signal.SIGINT)
        server.shutdown()

    signal.signal(signal.SIGINT, sigint_handler) # catch SIGINT (ctrl-c)   
