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

# robot_ip = 'http://192.168.0.5:8080'
robot_ip = 'http://localhost:8080'
#  robot_ip = 'http://10.194.203.22:8080'
ws_port = 1234

model_name = "Motion/data/TRINA_world_seed.xml"

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
        self.mode = 'Kinematic'
        self.components = ['base','left_limb','right_limb','left_gripper']
        self.init_UI_state = {}
        self.dt = 0.02
        self.robot = MotionClient(address = robot_ip)
        self.robot.startServer(mode = self.mode, components = self.components,codename = 'seed')
        self.left_limb_active = ('left_limb' in self.components)
        self.right_limb_active = ('right_limb' in self.components)
        self.base_active = ('base' in self.components)
        self.left_gripper_active = ('left_gripper' in self.components)
        self.right_gripper_active = ('right_gripper' in self.components)
        self.torso_active = ('torso' in self.components)
        self.temp_robot_telemetry = {'leftArm':[0,0,0,0,0,0],'rightArm':[0,0,0,0,0,0]}
        # self.robot_client = MotionClient()
        time.sleep(2)
        # self.robot = Motion()
        self.UI_state = {}
        self.init_pos_left = {}
        self.cur_pos_left = {}
        self.init_pos_right = {}
        self.init_headset_orientation = {}
        self.cur_pos_right = {}
        self.startup = True
        res = self.robot.startup()
        if not res:
            return

        stateRecieverThread = threading.Thread(target=self._serveStateReciever)
        stateRecieverThread.start()   


    def _serveStateReciever(self):

        # inputFile = 'UIOutputs.data'
        # fd = open(inputFile, 'rb')
        # dataset = pickle.load(fd)
        # self.init_UI_state = dataset
        self.setRobotToDefault()
        time.sleep(3)
        while(True):
            if(self.startup == True & (self.server["UI_STATE"].read()!=0)):
                print('started the initial values for the variables')
                self.init_UI_state = self.server['UI_STATE'].read()
                self.startup = False
                if(self.left_limb_active):
                    self.init_pos_left = self.robot.sensedLeftEETransform()
                if(self.right_limb_active):   
                    self.init_pos_right = self.robot.sensedRightEETransform()
                # either way, we must register the headset initial orientation:
                # we start by obtaining it: 
                self.init_headset_orientation = self.treat_headset_orientation(self.init_UI_state['headSetPositionState']['deviceRotation'])
            while(True):
                # try:
                self.last_time = time.time()
                if(self.left_limb_active):
                    self.cur_pos_left = self.robot.sensedLeftEETransform()
                if(self.right_limb_active):
                    self.cur_pos_right = self.robot.sensedRightEETransform()
                self.UI_state = self.server['UI_STATE'].read()

                # print('\n\n\n\n',self.UI_state['headSetPositionState'],'\n\n\n\n')

                self.UIStateLogic()
                time.sleep(self.dt)

                # except:
                #     pass
        
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
        if('left_limb' in self.components):
            self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,2)
        if('right_limb' in self.components):
            self.robot.setRightLimbPositionLinear(rightUntuckedConfig,2)
        #self.robot.setRightEEInertialTransform([[[1,0,0],[0,1,0],[0,0,1]], self.init_pos_right],3)

    def UIStateLogic(self):
        if(type(self.UI_state)!= int):
            if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
                self.setRobotToDefault()
            if (self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True):
                print('\n\n\n\n resetting UI initial state \n\n\n\n\n')
                self.init_UI_state = self.UI_state
                self.init_headset_orientation = self.treat_headset_orientation(self.UI_state['headSetPositionState']['deviceRotation'])

            if(self.base_active):
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
        # print('entered position_control\n\n\n\n\n')
        if(self.left_limb_active):
            self.positionControlArm('left')
            self.temp_robot_telemetry['leftArm'] = self.robot.sensedLeftLimbPosition()
        if(self.right_limb_active):
            self.positionControlArm('right')
            self.temp_robot_telemetry['rightArm'] = self.robot.sensedRightLimbPosition()
        # print('final_time =',time.time() - start_time)
        # print('\n\n\n\n\n',self.robot.sensedLeftLimbPosition(),type(self.robot.sensedLeftLimbPosition()),'\n\n\n\n\n\n')

        self.server['robotTelemetry'] = self.temp_robot_telemetry

        
    def positionControlArm(self,side):
        assert (side in ['left','right']), "invalid arm selection"
        R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
        # R_cw_rw_t = np.array([[0,0,1],[-1,0,0],[0,1,0]])
        joystick = side+"Controller"
        # print("{} button pushed".format(side),self.UI_state["controllerButtonState"][joystick])
        if self.UI_state["controllerButtonState"][joystick]["squeeze"][1] > 0.5 :
            # print("{} button pushed".format(side))
            # print('moving arm')
            if(side == 'right'):
                [RR_rw_rh,RT_rw_rh] = self.init_pos_right
                curr_position = np.array(self.robot.sensedRightEETransform()[1])
                # print('\n\n\n {} \n\n\n '.format(self.robot.sensedRightEETransform()[1]))

            elif(side == 'left'):
                # print('\n\n\n gets to the left side here \n\n\n ')
                [RR_rw_rh,RT_rw_rh] = self.init_pos_left
                curr_position = np.array(self.robot.sensedLeftEETransform()[1])
            RT_rw_rh = np.array(RT_rw_rh)
            RT_cw_cc = np.array(self.UI_state["controllerPositionState"][joystick]["controllerPosition"])
            RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"][joystick]["controllerPosition"])
            # print('\n\n\n\n\n',self.init_headset_orientation.inv().as_dcm(),np.matmul(np.matmul(R_cw_rw,self.init_headset_orientation.inv().as_dcm()), np.subtract(RT_cw_cc,RT_cw_ch)),'\n\n\n\n\n')
            # print('\n\n\n\n\n',RT_cw_cc,self.init_headset_orientation.as_dcm(),'\n\n\n\n\n')
            RT_final = np.add(RT_rw_rh, np.matmul(np.matmul(R_cw_rw,self.init_headset_orientation.inv().as_dcm()), np.subtract(RT_cw_cc,RT_cw_ch).transpose())).tolist()
            print(RT_cw_cc,np.subtract(RT_cw_cc,RT_cw_ch),np.matmul(R_cw_rw,self.init_headset_orientation.inv().as_dcm()))

            # we now check if the "ideal" velocity is greater than 1 m/s

            # current position:
            # linear_velocity_vector = (RT_final - curr_position)*(self.dt)
            # if(np.linalg.norm(linear_velocity_vector)>0.95):
            #     RT_final = (curr_position + (linear_velocity_vector/np.linalg.norm(linear_velocity_vector))*0.95/self.dt).tolist()

            RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))
            # RR_rw_rh_T = R.from_dcm(np.transpose(RR_rw_rh.as_dcm()))
            

            # Transforming from left handed to right handed
            # we first read the quaternion
            init_quat = np.array(self.init_UI_state["controllerPositionState"][joystick]['controllerRotation'])
            # turn it into a left handed rotation vector
            # right_handed_init_quat = np.array([-init_quat[2],init_quat[0],-init_quat[1],-(np.pi/180)*init_quat[3]])
            # #transform it to right handed:

            # curr_quat = self.UI_state["controllerPositionState"][joystick]['controllerRotation']

            # right_handed_curr_quat = np.array([-curr_quat[2],curr_quat[0],-curr_quat[1],-(np.pi/180)*curr_quat[3]])
            # RR_cw_ch = R.from_rotvec(right_handed_init_quat[0:3]*right_handed_init_quat[3])
            # RR_cw_ch_T = RR_cw_ch.inv()
            # RR_cw_cc = R.from_rotvec(right_handed_curr_quat[0:3]*right_handed_curr_quat[3])
            R_cw_rw = R.from_dcm(R_cw_rw)

            # world_adjustment_matrix = R_cw_rw
            world_adjustment_matrix = R.from_dcm(np.eye(3))
            init_angle = (np.pi/180)*init_quat[3]

            # right_handed_init_vec = world_adjustment_matrix.apply(init_quat[:3])*init_angle
            right_handed_init_vec = np.array([init_quat[2],-init_quat[0],init_quat[1]])*(init_angle)
            #transform it to right handed:

            curr_quat = np.array(self.UI_state["controllerPositionState"][joystick]['controllerRotation'])
            curr_angle = (np.pi/180)*curr_quat[3]
            # right_handed_curr_vec =  world_adjustment_matrix.apply(curr_quat[:3])*curr_angle
            right_handed_curr_vec = np.array([curr_quat[2],-curr_quat[0],curr_quat[1]])*(curr_angle)

            # R_cw_rw = R.from_dcm(R_cw_rw)
            # print('quats',curr_quat,init_quat)
            RR_cw_ch = R.from_rotvec(right_handed_init_vec)
            RR_cw_ch_T = RR_cw_ch.inv()
            RR_cw_cc = R.from_rotvec(right_handed_curr_vec)
            relative_rotation_vec = (RR_cw_ch_T*RR_cw_cc).as_rotvec()
            unspin = np.array([[1,0,0],[0,0,1],[0,1,0]])
            secondary_correction = np.matmul(np.matmul(unspin,self.init_headset_orientation.as_dcm()),unspin.transpose())
            print('secondary correction',secondary_correction,self.init_headset_orientation.as_dcm())

            corrected_relative_rotation_vec = secondary_correction*relative_rotation_vec.transpose()
            print('corrected relative....',corrected_relative_rotation_vec)
            corrected_R = R.from_rotvec(corrected_relative_rotation_vec)
            # print((RR_cw_ch_T*RR_cw_cc).as_rotvec(),'\n\n\n')
            # RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
            # RR_final = (RR_rw_rh*secondary_correction*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
            # RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
            RR_final = (RR_rw_rh*corrected_R).as_dcm().flatten().tolist()

            # RR_final = RR_rw_rh.as_dcm().flatten().tolist()
            start_time = time.process_time()
            if(side == 'right'):
                print('\n\n\n\n\n\n moving right arm \n\n\n\n\n\n\n')

                self.robot.setRightEEInertialTransform([RR_final,RT_final],self.dt)

            else:
                # print('moving left arm \n\n\n')
                self.robot.setLeftEEInertialTransform([RR_final,RT_final],self.dt)
                if((self.mode == 'Physical') and self.left_gripper_active):
                    closed_value = self.UI_state["controllerButtonState"]["leftController"]["squeeze"][0]*2
                    if(closed_value >= 0.1):                        
                        self.robot.setLeftGripperPosition([closed_value,closed_value,closed_value,0])
                    else:
                        self.robot.setLeftGripperPosition([0,0,0,0])

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
            RR_hs => Initial headset 

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
        # self.server['robotTelemetry'] = {'leftArm':self.robot.sensedLeftLimbPosition(),
        #     'rightArm':self.robot.sensedRightLimbPosition()}

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
            RR_final = (RR_rw_rh*self.init_headset_orientation.inv()*RR_cw_ch_T*RR_cw_cc).as_rotvec()

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
    def treat_headset_orientation(self,headset_orientation):
        """
        input: Rotvec of the headset position
        output: Rotation Matrix for the headset that ignores pitch and roll
        """
        #first we turn the input into a right_handed rotvec
        right_handed_rotvec =  np.array([headset_orientation[2],-headset_orientation[0],headset_orientation[1],(np.pi/180)*headset_orientation[3]])
        # right_handed_rotvec =  np.array([-headset_orientation[2],-headset_orientation[0],headset_orientation[1],-(np.pi/180)*headset_orientation[3]])

        partial_rotation = R.from_rotvec(right_handed_rotvec[:3]*right_handed_rotvec[3])
        # we then get its equivalent row, pitch and yaw 
        rpy = partial_rotation.as_euler('ZYX')
        # we then ignore its roll and pitch in unity
        rotation_final = R.from_euler('ZYX',[0,rpy[0],0])
        print(rotation_final.as_dcm())
        return rotation_final

       

if __name__ == "__main__" : 
    my_controller = UIController()
