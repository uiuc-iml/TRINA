import time,math
# from klampt import vis
# from klampt import WorldModel
# from klampt.model.trajectory import Trajectory
import threading
from Motion.motion_client_python3 import MotionClient
import json
from multiprocessing import Process, Manager, Pipe
import pickle
from pdb import set_trace
from numpy import linalg as LA
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import datetime
import csv 
from threading import Thread
import sys
import json
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback

# robot_ip = 'http://10.194.203.22:8080'
robot_ip = 'http://localhost:8080'
ws_port = 1234

model_name = "Motion/data/TRINA_world_reflex.xml"

roomname = "The Lobby"
zonename = "BasicExamples"
userId=0
roomId=-1
is_closed=0

class UIController:
    """handles UI_state and motion controller logic
    """
    def __init__(self):
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["UI_STATE"] = 0

        self.init_UI_state = {}
        self.dt = 0.1
        self.robot = MotionClient(address = robot_ip)
        self.robot.startServer(mode = 'Kinematic', components = ['left_limb'])
        # self.robot_client = MotionClient()
        # self.robot = Motion()
        self.UI_state = {}
        self.init_pos_left = {}
        self.cur_pos_left = {}
        self.init_pos_right = {}
        self.cur_pos_right = {}
        self.startup = True
        res = self.robot.startup()
        # if not res:
        #     return
        # world = WorldModel()
        # res = world.readFile(model_name)
        # if not res:
        #     raise RuntimeError("Unable to load Klamp't model")
        # self.vis_robot = world.robot(0)

        # visualUpdateThread = Process(target = self._visualUpdateLoop)
        # visualUpdateThread.start()

        stateRecieverThread = threading.Thread(target=self._serveStateReciever)
        stateRecieverThread.start()



        # vis.add("world",world)
        # vis.show()        


    # def _visualUpdateLoop(self):
    #     while True:
    #         try:
    #             q = self.robot_client.getKlamptSensedPosition()
    #         except:
    #             print("getKlamptSensedPosition failed")
    #             pass
    #         self.vis_robot.setConfig(q)
    #         EndLink = self.vis_robot.link(42)           # This link number should be the end effector link number
    #         Tlink = EndLink.getTransform()
    #         vis.add("Frame",Tlink)

    #         time.sleep(self.dt)

    def _serveStateReciever(self):

        # inputFile = 'UIOutputs.data'
        # fd = open(inputFile, 'rb')
        # dataset = pickle.load(fd)
        # self.init_UI_state = dataset
        self.setRobotToDefault()
        time.sleep(7.5)
        while(True):
            if(self.startup == True & (self.server["UI_STATE"].read()!=0)):
                print('started the initial values for the variables')
                self.init_UI_state = self.server['UI_STATE'].read()
                self.startup = False
                self.init_pos_left = self.robot.sensedLeftEETransform()
                # self.init_pos_right = self.robot.sensedRightEETransform()
            
            while(True):
                # try:https://docs.scipy.org/doc/numpy/reference/generated/numpy.linalg.norm.html
                time.sleep(0.025)

                # except:
                #     pass


        # return [self.robot.sensedLeftLimbPosition(),self.robot.sensedLeftLimbVelocity(),self.robot.sensedRightLimbPosition(),self.robot.sensedRightLimbVelocity()]
        return [self.robot.sensedLeftLimbPosition(),self.robot.sensedRightLimbPosition()]

    def moveRobotTest(self):
        # self.robot.setBaseVelocity([0,0])
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
        init_leftUntuckedConfig = [0,0,0,0,0,0] #motionAPI format
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,7)
        # self.robot.setBaseTargetPosition([0,0,0],[0.5,0.5])
        self.robot.setLeftLimbPositionLinear(init_leftUntuckedConfig,7)

    def setRobotToDefault(self):
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
        self.robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
        #self.robot.setRightEEInertialTransform([[[1,0,0],[0,1,0],[0,0,1]], self.init_pos_right],3)

    def UIStateLogic(self):
        if(type(self.UI_state)!= int):
            if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
                self.setRobotToDefault()
            if (self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True):
                print('\n\n\n\n resetting UI initial state \n\n\n\n\n')
                self.init_UI_state = self.UI_state
            self.baseControl()
            self.positionControl()
            #self.logTeleoperation('test')



    def baseControl(self):
        '''controlling base movement'''
        base_velocity = [0.5*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.5*(-self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][0])]
        # print(base_velocity)
        try:
            self.robot.setBaseVelocity(base_velocity)
        except:
            print("setBaseVelocity not successful")
            pass

    def positionControl(self):
        '''controlling arm movement with position command
        variable naming:
            LT_cw_cc => Left Traslational matrix from Controller World to Controller Current 
            RR_rw_rh => Right Rotational matrix from Robot World to Robot Home
            RR_rw_rh_T => Right Rotational matrix from Robot World to Robot Home Transpose
            R_cw_rw  => Rotational matrix from Controller World to Robot World

            cc------controller current
            rc------robot current

            cw------controller world
            rw------robot world

            ch------controller homw
            rh------robot home
        '''
        self.positionControlArm('left')
        self.positionControlArm('right')
        # print('final_time =',time.time() - start_time)
        # print('\n\n\n\n\n',self.robot.sensedLeftLimbPosition(),type(self.robot.sensedLeftLimbPosition()),'\n\n\n\n\n\n')
        self.server['robotTelemetry'] = {'leftArm':self.robot.sensedLeftLimbPosition(),
            'rightArm':self.robot.sensedRightLimbPosition()}

        
    def positionControlArm(self,side):
        assert (side in ['left','right']), "invalid arm selection"
        R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
        joystick = side+"Controller"
        # print("{} button pushed".format(side),self.UI_state["controllerButtonState"][joystick])
        if self.UI_state["controllerButtonState"][joystick]["squeeze"][0] > 0.5 :
            # print("{} button pushed".format(side))
            # print('moving arm')
            if(side == 'right'):
                [RR_rw_rh,RT_rw_rh] = self.init_pos_right
                curr_position = np.array(self.robot.sensedRightEETransform()[1])
            else:
                [RR_rw_rh,RT_rw_rh] = self.init_pos_left
                curr_position = np.array(self.robot.sensedLeftEETransform()[1])
            RT_rw_rh = np.array(RT_rw_rh)
            RT_cw_cc = np.array(self.UI_state["controllerPositionState"][joystick]["controllerPosition"])
            RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"][joystick]["controllerPosition"])

            RT_final = np.add(RT_rw_rh, np.matmul(R_cw_rw, np.subtract(RT_cw_cc,RT_cw_ch)))


            # we now check if the "ideal" velocity is greater than 1 m/s

            #current position:
            linear_velocity_vector = (RT_final - curr_position)*(1.0/60)
            if(np.linalg.norm(linear_velocity_vector)>0.95):
                RT_final = (curr_position + (linear_velocity_vector/np.linalg.norm(linear_velocity_vector))*0.95*60).tolist()
            # print(np.subtract(RT_cw_cc,RT_cw_ch),RT_rw_rh)
            
            # RR_unit_x = R.from_rotvec(np.pi/2 * np.array([1, 0, 0]))
            # RR_unit_y = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
            # RR_unit_z = R.from_rotvec(np.pi/2 * np.array([0, 0, 1]))

            RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))
            # RR_rw_rh_T = R.from_dcm(np.transpose(RR_rw_rh.as_dcm()))
            

            # Transforming from left handed to right handed
            # we first read the quaternion
            init_quat = self.init_UI_state["controllerPositionState"][joystick]['controllerRotation']
            # turn it into a left handed rotation vector
            right_handed_init_quat = np.array([-init_quat[2],init_quat[0],-init_quat[1],-(np.pi/180)*init_quat[3]])
            #transform it to right handed:

            curr_quat = self.UI_state["controllerPositionState"][joystick]['controllerRotation']

            right_handed_curr_quat = np.array([-curr_quat[2],curr_quat[0],-curr_quat[1],-(np.pi/180)*curr_quat[3]])
            RR_cw_ch = R.from_rotvec(right_handed_init_quat[0:3]*right_handed_init_quat[3])
            RR_cw_ch_T = R.from_dcm(RR_cw_ch.as_dcm().transpose())
            RR_cw_cc = R.from_rotvec(right_handed_curr_quat[0:3]*right_handed_curr_quat[3])

            # print((RR_cw_ch_T*RR_cw_cc).as_rotvec(),'\n\n\n')
            # RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
            RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
            start_time = time.process_time()
            if(side == 'right'):
                # print('moving right arm\n\n\n')

                self.robot.setRightEEInertialTransform([RR_final,RT_final],0.025)


            else:
                # print('moving left arm \n\n\n')
                self.robot.setLeftEEInertialTransform([RR_final,RT_final],0.025)
            # print('operation_time =',time.process_time() - start_time)




    # def velocityControl(self):
    #     '''still in progress'''
    #     # velocity command:  (newT - cur_leftR + init_leftR - orgT)/max(float(1/90),time.time()-self.last_time)
    #     try:
    #         if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
    #             [_,init_leftT] = self.init_pos_left
    #             [_,cur_leftT] = self.cur_pos_left
    #             newT = self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
    #             orgT = self.init_UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
    #             new_controller_T,org_controller_T = [newT[2],-newT[0],newT[1]],[orgT[2],-orgT[0],orgT[1]]
    #             cur_time = time.time()
    #             V = [(a-b+c-d)/max((1.0/90),cur_time - self.last_time) for a,b,c,d in zip(new_controller_T,cur_leftT,init_leftT,org_controller_T)]
    #             norm = LA.norm(np.array(V))

    #             direction = [a/norm for a in V]
    #             V = [float(a*min(norm,1)) for a in direction]

                
    #             self.last_time  =  cur_time
    #             if(norm >= 0.2):
    #                 self.robot.setLeftEEVelocity(V,[0,0,0])
    #     except Exception as e: 
    #         print(e)
    #         pass
    def velocityControl(self):
        '''controlling arm movement with position command
        variable naming:
            LT_cw_cc => Left Traslational matrix from Controller World to Controller Current 
            RR_rw_rh => Right Rotational matrix from Robot World to Robot Home
            RR_rw_rh_T => Right Rotational matrix from Robot World to Robot Home Transpose
            R_cw_rw  => Rotational matrix from Controller World to Robot World

            cc------controller current
            rc------robot current

            cw------controller world
            rw------robot world

            ch------controller homw
            rh------robot home
        '''
        self.velocityControlArm('left')
        self.velocityControlArm('right')
        # print('final_time =',time.time() - start_time)
        # print('\n\n\n\n\n',self.robot.sensedLeftLimbPosition(),type(self.robot.sensedLeftLimbPosition()),'\n\n\n\n\n\n')
        self.server['robotTelemetry'] = {'leftArm':self.robot.sensedLeftLimbPosition(),
            'rightArm':self.robot.sensedRightLimbPosition()}

    def velocityControlArm(self,side):
        assert (side in ['left','right']), "invalid arm selection"

        # we start by calculating the desired rotation and position vectors, RR_final and RT_final
        R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
        joystick = side+"Controller"
        # print("{} button pushed".format(side),self.UI_state["controllerButtonState"][joystick])
        if self.UI_state["controllerButtonState"][joystick]["squeeze"][0] > 0.5 :
            # print("{} button pushed".format(side))
            # print('moving arm')
            if(side == 'right'):
                [RR_rw_rh,RT_rw_rh] = self.init_pos_right
            else:
                [RR_rw_rh,RT_rw_rh] = self.init_pos_left
            RT_rw_rh = np.array(RT_rw_rh)
            RT_cw_cc = np.array(self.UI_state["controllerPositionState"][joystick]["controllerPosition"])
            RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"][joystick]["controllerPosition"])

            RT_final = np.add(RT_rw_rh, np.matmul(R_cw_rw, np.subtract(RT_cw_cc,RT_cw_ch))).tolist()


            RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))
            

            # Transforming from left handed to right handed
            # we first read the quaternion
            init_quat = self.init_UI_state["controllerPositionState"][joystick]['controllerRotation']
            # turn it into a left handed rotation vector
            right_handed_init_quat = np.array([-init_quat[2],init_quat[0],-init_quat[1],-(np.pi/180)*init_quat[3]])
            #transform it to right handed:

            curr_quat = self.UI_state["controllerPositionState"][joystick]['controllerRotation']

            right_handed_curr_quat = np.array([-curr_quat[2],curr_quat[0],-curr_quat[1],-(np.pi/180)*curr_quat[3]])
            RR_cw_ch = R.from_rotvec(right_handed_init_quat[0:3]*right_handed_init_quat[3])
            RR_cw_ch_T = R.from_dcm(RR_cw_ch.as_dcm().transpose())
            RR_cw_cc = R.from_rotvec(right_handed_curr_quat[0:3]*right_handed_curr_quat[3])

            # print((RR_cw_ch_T*RR_cw_cc).as_rotvec(),'\n\n\n')
            # RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
            RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_rotvec()

            # we now calculate the error values:

            #we first get the current end-effector position

            if(side == 'right'):
                # print('moving right arm\n\n\n')

                self.robot.setRightEEInertialTransform([RR_final,RT_final],0.025)


            else:
                # print('moving left arm \n\n\n')
                self.robot.setLeftEEInertialTransform([RR_final,RT_final],0.025)
            # print('operation_time =',time.process_time() - start_time)

    def logTeleoperation(self,name):
        q = self.robotPoser.get()
        left_limb_command = q[10:16]
        right_limb_command = q[35:41]
        self.fileName = 'Teleoperation_log/motion' + ''.join(name) + '.csv'
        self.saveStartTime =  datetime.datetime.utcnow()
        self.saveEndTime = datetime.datetime.utcnow() + datetime.timedelta(0,3.1)
        fields = ['timestep', 'Left Shoulder', 'Left UpperArm', 'Left ForeArm', 'Left Wrist1','Left Wrist2','Left Wrist3','Right Shoulder', 'Right UpperArm', 'Right ForeArm', 'Right Wrist1','Right Wrist2','Right Wrist3','Left EE Transform', 'Right EE Transform' ] 
        with open(self.fileName, 'w') as csvfile: 
            # creating a csv writer object 
            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow(fields)
        self.robot.setLeftLimbPositionLinear(left_limb_command,3)
        self.robot.setRightLimbPositionLinear(right_limb_command,3)
        return
    
       

if __name__ == "__main__" : 
    my_controller = UIController()


