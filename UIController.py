import time,math
from klampt import vis
from klampt import WorldModel
import threading
from Motion.motion_client import MotionClient
from Motion.motion import Motion
import json
from multiprocessing import Process, Manager
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
from UIStateReciever import UIStateReciever
import pickle
from pdb import set_trace
import time,math
from numpy import linalg as LA
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import datetime
import csv 


robot_ip = '130.126.139.236'
ws_port = 1234

model_name = "Motion/data/TRINA_world_reflex.xml"

class UIController:
    """handles UI_state and motion controller logic
    """
    def __init__(self):
        self.init_UI_state = {}
        self.dt = 0.02
        self.robot = MotionClient()
        self.robot_clent = MotionClient()
        self.UI_state = {}
        self.init_pos_left = {}
        self.cur_pos_left = {}
        self.init_pos_right = {}
        self.cur_pos_right = {}
        res = self.robot.startup()
        if not res:
            return
        world = WorldModel()
        res = world.readFile(model_name)
        if not res:
            raise RuntimeError("Unable to load Klamp't model")
        self.vis_robot = world.robot(0)

        visualUpdateThread = threading.Thread(target = self._visualUpdateLoop)
        visualUpdateThread.start()

        stateRecieverThread = threading.Thread(target=self._serveStateReciever)
        stateRecieverThread.start()

        vis.add("world",world)
        vis.show()

    def _visualUpdateLoop(self):
        while True:
            try:
                q = self.robot_clent.getKlamptSensedPosition()
            except:
                print("getKlamptSensedPosition failed")
                pass
            self.vis_robot.setConfig(q)
            time.sleep(self.dt)

    def _serveStateReciever(self):
        inputFile = 'UIOutputs.data'
        fd = open(inputFile, 'rb')
        dataset = pickle.load(fd)
        self.init_UI_state = dataset
        self.init_pos_left = self.robot.sensedLeftEETransform()
        self.init_pos_right = self.robot.sensedRightEETransform()

        while True:
            try:
                self.last_time = time.time()
                self.cur_pos_left = self.robot.sensedLeftEETransform()
                self.cur_pos_right = self.robot.sensedRightEETransform()
                inputFile = 'UIOutputs.data'
                fd = open(inputFile, 'rb')
                dataset = pickle.load(fd)
                self.UI_state = dataset
                self.UIStateLogic()
                time.sleep(0.2)
            except:
                pass


    def moveRobotTest(self):
        self.robot.setBaseVelocity([0,0])
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
        init_leftUntuckedConfig = [0,0,0,0,0,0] #motionAPI format
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
        self.robot.setBaseTargetPosition([0,0,0],[0.5,0.5])
        self.robot.setLeftLimbPositionLinear(init_leftUntuckedConfig,5)

    def setRobotToDefualt(self):
        # leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        # rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        # self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,1)
        # self.robot.setRightLimbPositionLinear(rightUntuckedConfig,1)
        self.robot.setRightEEInertialTransform([[[1,0,0],[0,1,0],[0,0,1]], self.init_pos_right],3)

    def UIStateLogic(self):
        if self.UI_state["controllerButtonState"]["leftController"]["press"][1] == True :
            self.setRobotToDefualt()
        self.baseControl()
        self.positionControl()



    def baseControl(self):
        '''controlling base movement'''
        base_velocity = [0.5*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.5*(-self.UI_state["controllerButtonState"]["leftController"]["thumbstickMovement"][0])]
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
        R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
        # try:
        #     if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
        #         [leftR,leftT] = self.init_pos_left
        #         newR = R.from_quat(self.UI_state["controllerPositionState"]["leftController"]["controllerOrientation"])
        #         newR = newR.as_dcm().flatten().tolist()
        #         newT = self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
        #         orgT = self.init_UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
        #         T = [leftT[0]+(newT[2]-orgT[2]),leftT[1]-(newT[0]-orgT[0]),leftT[2]+(newT[1]-orgT[1])]
        #         self.robot.setLeftEEInertialTransform([leftR,T],0.02)
        # except Exception as e: 
        #     print(e)
        #     pass
           
        try:
            if self.UI_state["controllerButtonState"]["rightController"]["press"][0] == True :
                [RR_rw_rh,RT_rw_rh] = self.init_pos_right
                RT_rw_rh = np.array(RT_rw_rh)
                RT_cw_cc = np.array(self.UI_state["controllerPositionState"]["rightController"]["controllerPosition"])
                RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"]["rightController"]["controllerPosition"])

                RT_final = np.add(RT_rw_rh, np.matmul(R_cw_rw, np.subtract(RT_cw_cc,RT_cw_ch))).tolist()
                
                RR_unit_x = R.from_rotvec(np.pi/2 * np.array([1, 0, 0]))
                RR_unit_y = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
                RR_unit_z = R.from_rotvec(np.pi/2 * np.array([0, 0, 1]))

                RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))
                RR_rw_rh_T = R.from_dcm(np.transpose(RR_rw_rh.as_dcm()))
                
                # RR_cw_cc = R.from_quat(self.UI_state["controllerPositionState"]["rightController"]["controllerOrientation"])
                # RR_cw_cc = R.from_dcm(RR_cw_cc.as_dcm())
                # Transforming from left handed to right handed
                init_quat = self.init_UI_state["      "]["rightController"]["controllerOrientation"]
                right_handed_init_quat = np.array([-init_quat[2],init_quat[0],-init_quat[1],init_quat[3]])
                curr_quat = self.UI_state["controllerPositionState"]["rightController"]["controllerOrientation"]
                right_handed_curr_quat = np.array([-curr_quat[2],curr_quat[0],-curr_quat[1],curr_quat[3]])
                #RR_cw_ch = R.from_quat(self.init_UI_state["controllerPositionState"]["rightController"]["controllerOrientation"])
                RR_cw_ch = R.from_quat(right_handed_init_quat)
                RR_cw_ch_T = R.from_dcm(np.transpose(RR_cw_ch.as_dcm()))
                RR_cw_cc = R.from_quat(right_handed_curr_quat)
                print(self.init_UI_state["controllerPositionState"]["rightController"]["controllerOrientation"],'\n\n\n\n\n\n')

                """ 
                if self.UI_state["controllerButtonState"]["rightController"]["press"][1] == True :
                    RR_cw_cc = RR_cw_ch*RR_unit_x
                elif self.UI_state["controllerButtonState"]["rightController"]["press"][2] == True :
                    RR_cw_cc = RR_cw_ch*RR_unit_y
                elif self.UI_state["controllerButtonState"]["rightController"]["press"][3] == True :
                    RR_cw_cc = RR_cw_ch*RR_unit_z
                else:
                    return """

                RR_final = (RR_rw_rh*RR_cw_ch_T*RR_cw_cc).as_dcm().flatten().tolist()
                self.robot.setRightEEInertialTransform([RR_cw_cc.as_dcm().flatten().tolist(),RT_final],0.025)
        

        

        except Exception as e: 
            print(e)
            pass

    def velocityControl(self):
        '''still in progress'''
        # velocity command:  (newT - cur_leftR + init_leftR - orgT)/max(float(1/90),time.time()-self.last_time)
        try:
            if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
                [_,init_leftT] = self.init_pos_left
                [_,cur_leftT] = self.cur_pos_left
                newT = self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
                orgT = self.init_UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
                new_controller_T,org_controller_T = [newT[2],-newT[0],newT[1]],[orgT[2],-orgT[0],orgT[1]]
                cur_time = time.time()
                V = [(a-b+c-d)/max((1.0/90),cur_time - self.last_time) for a,b,c,d in zip(new_controller_T,cur_leftT,init_leftT,org_controller_T)]
                norm = LA.norm(np.array(V))

                direction = [a/norm for a in V]
                V = [float(a*min(norm,1)) for a in direction]

                
                self.last_time  =  cur_time
                if(norm >= 0.2):
                    self.robot.setLeftEEVelocity(V,[0,0,0])
        except Exception as e: 
            print(e)
            pass

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
    a = UIController()