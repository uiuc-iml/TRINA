import time,math,datetime
import threading
# from motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import numpy as np
from scipy.spatial.transform import Rotation as R
import os,csv,sys,shlex
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback
from multiprocessing import Pool, TimeoutError,Process
import trina_modules
import sys, inspect
import atexit
import subprocess
from TRINAConfig import *
from sensorModule2 import Camera_Robot
from klampt import WorldModel,Geometry3D
import rosgraph
if(sys.version_info[0] < 3):
    from future import *
    from motion_client import MotionClient
else:
    from motion_client_python3 import MotionClient
    from importlib import reload
from jarvis import Jarvis

robot_ip = 'http://localhost:8080'

ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

class CommandServer:

    def __init__(self,components =  ['base','left_limb','right_limb','left_gripper'], robot_ip = robot_ip, model_name = model_name,mode = 'Kinematic',world_file = './data/TRINA_world_anthrax_PointClick.xml'):
        # we first check if redis is up and running:
        try:
            self.interface = RedisInterface(host="localhost")
            self.interface.initialize()
            self.server = KeyValueStore(self.interface)
            print('Reem already up and running, skipping creation process')
        except Exception as e:
            # if we cannot connect to redis, we start the server for once:
            print('starting redis server because of ',e)
            self.redis_process = Process(target = self.start_redis())
            self.redis_process.daemon = False
            self.redis_process.start()
            # self.start_redis()
            # wait for it to start
            time.sleep(2)
            # then we start our connections as normal:
            self.interface = RedisInterface(host="localhost")
            self.interface.initialize()
            self.server = KeyValueStore(self.interface)
        
        self.start_ros_stuff()
        self.world_file = world_file
        # we then proceed with startup as normal

        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["ROBOT_STATE"] = 0
        self.server['ROBOT_COMMAND'] = {}
        self.server['HEALTH_LOG'] = {}
        self.server['ACTIVITY_STATUS'] = {}
        self.mode = mode
        self.components = components
        self.init_robot_state = {}
        self.dt = 0.001
        self.robot = MotionClient(address = robot_ip)
        # self.controller = UIController()
        self.robot.restartServer(mode = self.mode, components = self.components,codename = 'anthrax_lowpoly')
        self.robot_state = {}
        self.robot_command = {}
        self.modules = ["Wipe", "UIModule", "Motion"]
        self.startup = True
        self.robot_active = True
        self.shut_down_flag = False
        self.left_limb_active = ('left_limb' in self.components)
        self.right_limb_active = ('right_limb' in self.components)
        self.base_active = ('base' in self.components)
        self.left_gripper_active = ('left_gripper' in self.components)
        self.right_gripper_active = ('right_gripper' in self.components)
        self.torso_active = ('torso' in self.components)
        self.query_robot = MotionClient(address = robot_ip)
        # self.controller = UIController()
        self.query_robot.startServer(mode = self.mode, components = self.components,codename = 'anthrax_lowpoly')
        self.query_robot.startup()
        res = self.robot.startup()
        if not res:
            print('Failed!')
        self.init_robot_states()
        time.sleep(0.2)
        if(self.mode == 'Kinematic'):
            self.world = WorldModel()
            self.world.readFile(self.world_file )
            self.sensor_module = Camera_Robot(robot = self.robot,world = self.world)
            time.sleep(3)
        self.health_dict = {}
        # create the list of threads
        self.modules_dict = {}

        manager = Manager()

        self.active_modules = manager.dict()
        self.active_modules['UI'] = True
        self.start_modules()


        stateRecieverThread = threading.Thread(target=self.stateReciever)
        stateRecieverThread.start()
        commandRecieverThread = Process(target=self.commandReciever, args=(self.robot, self.active_modules))
        commandRecieverThread.daemon = True
        commandRecieverThread.start()
        moduleMonitorThread = threading.Thread(target=self.moduleMonitor)
        moduleMonitorThread.start()
        atexit.register(self.shutdown_all)

        # self.switch_module_activity(['C2'])
        # self.empty_command.update({'UI':[]})

    def init_robot_states(self):

        pos_left = {}
        pos_right = {}
        pos_base = {}
        pos_left_gripper = {}
        pos_right_gripper = {}
        pos_torso = {}
        vel_base = {}
        vel_right = {}
        vel_left = {}
        try:
            if(self.left_limb_active):
                pos_left = self.query_robot.sensedLeftEETransform()
                # print("left position")
                vel_left = self.query_robot.sensedLeftEEVelocity()
                # print("left velocity")
            if(self.right_limb_active):
                pos_right = self.query_robot.sensedRightEETransform()
                # print("right position")
                vel_right = self.query_robot.sensedRightEEVelocity()
                # print("right velocity")
            if(self.base_active):
                pos_base = self.query_robot.sensedBasePosition()
                # print("base position")
                vel_base = self.query_robot.sensedBaseVelocity()
            klampt_q = get_klampt_model_q('anthrax',left_limb = self.query_robot.sensedLeftLimbPosition(), right_limb = self.query_robot.sensedRightLimbPosition(), base = pos_base)
                # print("base velocity")
            # if(self.left_gripper_active):
            #     pos_left_gripper = self.robot.sensedLeftGripperPosition()
            #     print("left gripper position")
            # if(self.right_gripper_active):
            #     pos_right_gripper = self.robot.sensedRightGripperPosition()
            #     print("right gripper position")
            # if(self.torso_active):
            #     pos_torso = self.robot.sensedTorsoPosition()
            #     print("torso position")
        except Exception as e:
            print(e)
        UI_state = self.server["UI_STATE"].read()
        # build the state.
        self.server["ROBOT_STATE"] = {
                                "Position" : {
                                    "LeftArm" : pos_left,
                                    "RightArm" : pos_right,
                                    "Base" : pos_base,
                                    "Torso": pos_torso,
                                    "LeftGripper" : pos_left_gripper,
                                    "RightGripper" : pos_right_gripper,
                                    "Robotq": klampt_q
                                    },
                                "Velocity" : {
                                    "LeftArm" : vel_left,
                                    "RightArm" : vel_right,
                                    "Base" : vel_base
                                }
                                }


    def start_module(self,module,name):
        module_jarvis = Jarvis(name,self.sensor_module)
        a = module(module_jarvis)
        return a.return_processes()

    def start_modules(self,module_names = []):
        import trina_modules
        trina_modules = reload(trina_modules)
        activity_dict = {}
        command_dict = {}
        try:
            if(module_names == []):
                for name, obj in inspect.getmembers(trina_modules):
                    if inspect.isclass(obj):
                        if(str(obj).find('trina_modules') != -1):
                            tmp = self.start_module(obj,name)
                            self.modules_dict.update({name:tmp})
                            self.health_dict.update({name:[True,time.time()]})
                            activity_dict.update({name:'idle'})
                            command_dict.update({name:[]})
                            if(name != 'UI'):
                                self.active_modules[name] = False
                            else:
                                self.active_modules[name] = True

                self.server['HEALTH_LOG'] = self.health_dict
                self.server['ROBOT_COMMAND'] = command_dict
                self.server['ACTIVITY_STATUS'] = activity_dict
                self.empty_command = command_dict
            else:
                print('starting only modules '+ str(module_names))
                for name, obj in inspect.getmembers(trina_modules):
                    if inspect.isclass(obj):
                        if(str(obj).find('trina_modules') != -1):
                            if(name in module_names):
                                print('killing module '+ name)
                                for pcess in self.modules_dict[name]:
                                    pcess.terminate()
                                self.modules_dict.update({name:[]})
                                print('restarting only module ' + name)
                                tmp = self.start_module(obj,name)
                                self.modules_dict.update({name:tmp})
                                self.server['HEALTH_LOG'][name] = [True,time.time()]
                                self.server['ACTIVITY_STATUS'][name] = 'idle'
                                if(self.active_modules[name]):
                                    self.active_modules[name] = False
        except Exception as e:
            print('Failed to initialize module',name,'due to ',e)
    def switch_module_activity(self,to_activate,to_deactivate = []):
        print('switching module activity:')
        if(to_deactivate == []):
            tmp = self.server['ACTIVITY_STATUS'].read()
            for i in tmp.keys():
                # print(i)
                self.server['ACTIVITY_STATUS'][str(i)] = 'idle'
                if(self.active_modules[i]):
                    self.active_modules[i] = False
        else:
            print('gets here')

            for i in to_deactivate:
                self.server['ACTIVITY_STATUS'][i] = 'idle'
                if(self.active_modules[i]):
                    self.active_modules[i] = False
        for i in to_activate:
            self.server['ACTIVITY_STATUS'][i] = 'active'
            self.active_modules[i] = True

    def shutdown_all(self):
        self.shutdown()
        print('closing all and exiting')
        for module in self.modules_dict.keys():
            for pcess in self.modules_dict[module]:
                try:
                    pcess.terminate()
                except Exception as e:
                    print(e)
                    pass
    #this is place holder for moduleMonitor
    def activate(self,name):
        while not self.shut_down_flag:
            time.sleep(0.1)

    def stateReciever(self):
        pos_left = {}
        pos_right = {}
        pos_base = {}
        pos_left_gripper = {}
        pos_right_gripper = {}
        pos_torso = {}
        vel_base = {}
        vel_right = {}
        vel_left = {}
        loopStartTime = time.time()
        while not self.shut_down_flag:
            self.robot_state = self.server['ROBOT_STATE'].read()
            self.startup = False
            try:
                if(self.left_limb_active):
                    pos_left = self.query_robot.sensedLeftEETransform()
                    # print("left position")
                    vel_left = self.query_robot.sensedLeftEEVelocity()
                    # print("left velocity")
                if(self.right_limb_active):
                    pos_right = self.query_robot.sensedRightEETransform()
                    # print("right position")
                    vel_right = self.query_robot.sensedRightEEVelocity()
                    # print("right velocity")
                if(self.base_active):
                    pos_base = self.query_robot.sensedBasePosition()
                    # print("base position")
                    vel_base = self.query_robot.sensedBaseVelocity()
                klampt_q = get_klampt_model_q('anthrax',left_limb = self.query_robot.sensedLeftLimbPosition(), right_limb = self.query_robot.sensedRightLimbPosition(), base = pos_base)
                # print("base velocity")
            # if(self.left_gripper_active):
            #     pos_left_gripper = self.robot.sensedLeftGripperPosition()
            #     print("left gripper position")
            # if(self.right_gripper_active):
            #     pos_right_gripper = self.robot.sensedRightGripperPosition()
            #     print("right gripper position")
            # if(self.torso_active):
            #     pos_torso = self.robot.sensedTorsoPosition()
            #     print("torso position")
            except Exception as e:
                print(e)
            UI_state = self.server["UI_STATE"].read()
            # build the state.
            self.server["ROBOT_STATE"] = {
                                    "Position" : {
                                        "LeftArm" : pos_left,
                                        "RightArm" : pos_right,
                                        "Base" : pos_base,
                                        "Torso": pos_torso,
                                        "LeftGripper" : pos_left_gripper,
                                        "RightGripper" : pos_right_gripper,
                                        "Robotq": klampt_q
                                        },
                                    "Velocity" : {
                                        "LeftArm" : vel_left,
                                        "RightArm" : vel_right,
                                        "Base" : vel_base,
                                    },
                                    }
            ################
            elapsedTime = time.time() - loopStartTime
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass
        print('\n\n\n\nstopped updating state!!! \n\n\n\n')

    def commandReciever(self,robot,active_modules):
        self.dt = 0.01
        self.robot = robot
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)     
        self.active_modules = active_modules   
        while(True):
            self.active_modules['UI'] = True
            loopStartTime = time.time()
            self.robot_command = self.server['ROBOT_COMMAND'].read()
            if(self.empty_command.keys() != self.robot_command.keys()):
                print('updating list of modules')
                empty_command = {}
                for key in self.robot_command.keys():
                    empty_command.update({str(key):[]})
                    try:
                        self.active_modules[str(key)]
                    except Exception as e:
                        if(str(key) != str('UI')):
                            self.active_modules[str(key)] = False
                        else:
                            self.active_modules[str(key)] = True
                self.empty_command = empty_command
            self.server['ROBOT_COMMAND'] = self.empty_command

            for i in self.robot_command.keys():
                if (self.robot_command[str(i)] != []):
                    if(self.active_modules[str(i)]):
                        commandList = self.robot_command[str(i)]
                        for command in commandList:
                            self.run(command)
                    else:
                        print('ignoring commands from ',str(i),self.robot_command[str(i)])
            elapsedTime = time.time() - loopStartTime
            # print('\n\n Frequency of execution loop:', 1/elapsedTime,'\n\n')
            if elapsedTime < self.dt:
                time.sleep(self.dt-elapsedTime)
            else:
                pass

    def run(self,command):
        try:
            exec(command)
        except Exception as e:
            print('there was an error executing your command!',e)
        finally:
            print("command recieved was " + command)

            pass


    #0 -> dead
    #1 -> healthy
    def moduleMonitor(self):
        self.monitoring_dt = 1
        self.tolerance = 15
        while not self.shut_down_flag:
            to_restart = []

            loopStartTime = time.time()
            for module in self.modules_dict.keys():
                moduleStatus = self.server["HEALTH_LOG"][module].read()
                if ((time.time()-moduleStatus[1]) > self.tolerance*self.monitoring_dt):
                    print("Module " + module + " is dead  due to timeout, queueing restart")
                    to_restart.append(module)
                else:
                    processes = self.modules_dict[module]
                    for pcess in processes:
                        if(not(pcess.is_alive())):
                            print("Module " + module + " is dead due to dead process, queueing restart")
                            to_restart.append(module)
                            break
            if(to_restart != []):
                print('restarting modules ' + str(to_restart))
                self.start_modules(to_restart)

            elapsedTime = time.time() - loopStartTime
            if elapsedTime < self.monitoring_dt:
                time.sleep(self.monitoring_dt-elapsedTime)
            else:
                pass


    # def sigint_handler(self, signum, frame):
    #     """ Catch Ctrl+C tp shutdown the robot

    #     """
    #     assert(signum == signal.SIGINT)
    #     # logger.warning('SIGINT caught...shutting down the api!')
    #     print("SIGINT caught...shutting down the api!")
    #     self.shutdown()

    def shutdown(self):
        #send shutdown to all modules
        self.shut_down_flag = True
        return 0


    def is_robot_active(self):
        left_limb_active = ('left_limb' in self.components)
        right_limb_active = ('right_limb' in self.components)
        base_active = ('base' in self.components)
        left_gripper_active = ('left_gripper' in self.components)
        right_gripper_active = ('right_gripper' in self.components)
        torso_active = ('torso' in self.components)

        return (left_limb_active and right_limb_active and base_active and left_gripper_active and right_gripper_active and torso_active)


    def setRobotToDefault(self):
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        print('right_Untucked',rightUntuckedConfig)
        if('left_limb' in self.components):
            self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,2)
        if('right_limb' in self.components):
            self.robot.setRightLimbPositionLinear(rightUntuckedConfig,2)

    def start_redis(self):
        print('starting redis')
        origWD = os.getcwd() # remember our original working directory
        #setting up the start of the redis server
        redis_server_path = os.path.expanduser('~/database-server/redis-5.0.4/src/redis-server')
        redis_conf_path = os.path.expanduser('~/database-server/redis.conf')
        redis_folder = os.path.expanduser('~/database-server')
        command_string = '{} {}'.format(redis_server_path,redis_conf_path)
        os.chdir(redis_folder)
        args = shlex.split(command_string)

        if(sys.version_info[0] < 3):
            pid=os.fork()
            if pid == 0:
                try:
                    os.setsid()
                except:
                    print('could not separate the process.')
                # if pid==0: # new process
                self.redis_pipe = subprocess.Popen(args)   
                while(True):
                    time.sleep(1000)
        else:

            self.redis_pipe = subprocess.Popen(args,start_new_session = True)

        # reverting back to trina directory
        os.chdir(origWD)
    
    def start_ros_stuff(self):
        print('starting ros stuff')
        origWD = os.getcwd() # remember our original working directory
        #setting up the start of the redis server
        catkin_folder = os.path.expanduser('~/catkin_ws/devel/')
        os.chdir(catkin_folder)
        # os.system('./setup.sh')
        # redis_conf_path = os.path.expanduser('~/database-server/redis.conf')
        # redis_folder = os.path.expanduser('~/database-server')
        command_string = 'roscore'
        gmapping_string = 'rosrun gmapping slam_gmapping scan:=base_scan _xmax:=10 _xmin:=-10 _ymax:=10 _ymin:=-10'
        ros_args = shlex.split(command_string)
        gmapping_args = shlex.split(gmapping_string)
        if(not rosgraph.is_master_online()):
            self.ros_process = subprocess.Popen(ros_args)
        else:
            print('roscore already running, skipping this part')
        time.sleep(3)

        self.gmapping = subprocess.Popen(gmapping_args)
        print('executed gmapping')
        os.chdir(origWD)

if __name__=="__main__":
    server = CommandServer()
    while(True):
        time.sleep(100)
        pass