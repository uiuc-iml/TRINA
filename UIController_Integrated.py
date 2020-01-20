import time,math
from klampt import vis
from klampt import WorldModel
from klampt.model.trajectory import Trajectory
import threading
from Motion.motion_client import MotionClient
from Motion.motion import Motion
import json
from multiprocessing import Process, Manager, Pipe
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
import pickle
from pdb import set_trace
import time,math
from numpy import linalg as LA
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
import datetime
import csv 
import time
import websocket
from threading import Thread
import time
import sys
import json


robot_ip = '130.126.139.236'
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
        self.init_UI_state = {}
        self.dt = 0.02
        self.robot = MotionClient()
        # self.robot_client = MotionClient()
        self.UI_state = {}
        self.init_pos_left = {}
        self.cur_pos_left = {}
        self.init_pos_right = {}
        self.cur_pos_right = {}
        self.startup = True
        res = self.robot.startup()
        if not res:
            return
        # world = WorldModel()
        # res = world.readFile(model_name)
        # if not res:
        #     raise RuntimeError("Unable to load Klamp't model")
        # self.vis_robot = world.robot(0)

        # visualUpdateThread = Process(target = self._visualUpdateLoop)
        # visualUpdateThread.start()

        # stateRecieverThread = threading.Thread(target=self._serveStateReciever)
        # stateRecieverThread.start()

        # vis.add("world",world)
        # vis.show()
        self.setRobotToDefault()

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

    def _serveStateReciever(self,UI_STATE):
        # inputFile = 'UIOutputs.data'
        # fd = open(inputFile, 'rb')
        # dataset = pickle.load(fd)
        # self.init_UI_state = dataset
        if(self.startup == True):
            self.init_UI_state = UI_STATE
            print("Initial UI STATE",UI_STATE)
            self.startup = False
            self.init_pos_left = self.robot.sensedLeftEETransform()
            self.init_pos_right = self.robot.sensedRightEETransform()

        else:
            try:
                self.last_time = time.time()
                self.cur_pos_left = self.robot.sensedLeftEETransform()
                self.cur_pos_right = self.robot.sensedRightEETransform()
                self.UI_state = UI_STATE

                # print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
                # print(UI_STATE)
                # inputFile = 'UIOutputs.data'
                # fd = open(inputFile, 'rb')
                # dataset = pickle.load(fd)
                # self.UI_state = dataset
                self.UIStateLogic()

            except:
                pass

        # return [self.robot.sensedLeftLimbPosition(),self.robot.sensedLeftLimbVelocity(),self.robot.sensedRightLimbPosition(),self.robot.sensedRightLimbVelocity()]
        return [self.robot.sensedLeftLimbPosition(),self.robot.sensedRightLimbPosition()]

    def moveRobotTest(self):
        self.robot.setBaseVelocity([0,0])
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
        init_leftUntuckedConfig = [0,0,0,0,0,0] #motionAPI format
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
        self.robot.setBaseTargetPosition([0,0,0],[0.5,0.5])
        self.robot.setLeftLimbPositionLinear(init_leftUntuckedConfig,5)

    def setRobotToDefault(self):
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,3)
        self.robot.setRightLimbPositionLinear(rightUntuckedConfig,3)
        #self.robot.setRightEEInertialTransform([[[1,0,0],[0,1,0],[0,0,1]], self.init_pos_right],3)

    def UIStateLogic(self):
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
        base_velocity = [0.5*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.5*(-self.UI_state["controllerButtonState"]["leftController"]["thumbstickMovement"][0])]
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
        R_cw_rw = np.array([[0,0,1],[-1,0,0],[0,1,0]])
        try:
            if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
                [leftR,leftT] = self.init_pos_left
                newR = R.from_quat(self.UI_state["controllerPositionState"]["leftController"][u'controller\u200bRotation'])
                newR = newR.as_dcm().flatten().tolist()
                newT = self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
                orgT = self.init_UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
                T = [leftT[0]+(newT[2]-orgT[2]),leftT[1]-(newT[0]-orgT[0]),leftT[2]+(newT[1]-orgT[1])]
                self.robot.setLeftEEInertialTransform([leftR,T],0.02)
        except Exception as e: 
            print(e)
            pass
           
        try:
            if self.UI_state["controllerButtonState"]["rightController"]["squeeze"][0] > 0.5 :
                [RR_rw_rh,RT_rw_rh] = self.init_pos_right
                RT_rw_rh = np.array(RT_rw_rh)
                RT_cw_cc = np.array(self.UI_state["controllerPositionState"]["rightController"]["controllerPosition"])
                RT_cw_ch = np.array(self.init_UI_state["controllerPositionState"]["rightController"]["controllerPosition"])

                RT_final = np.add(RT_rw_rh, np.matmul(R_cw_rw, np.subtract(RT_cw_cc,RT_cw_ch))).tolist()
                print(np.subtract(RT_cw_cc,RT_cw_ch),RT_rw_rh)
                
                RR_unit_x = R.from_rotvec(np.pi/2 * np.array([1, 0, 0]))
                RR_unit_y = R.from_rotvec(np.pi/2 * np.array([0, 1, 0]))
                RR_unit_z = R.from_rotvec(np.pi/2 * np.array([0, 0, 1]))

                RR_rw_rh = R.from_dcm((np.array(RR_rw_rh).reshape((3,3))))
                RR_rw_rh_T = R.from_dcm(np.transpose(RR_rw_rh.as_dcm()))
                
                # RR_cw_cc = R.from_quat(self.UI_state["controllerPositionState"]["rightController"][u'controller\u200bRotation'])
                # RR_cw_cc = R.from_dcm(RR_cw_cc.as_dcm())
                # Transforming from left handed to right handed
                # we first read the quaternion
                init_quat = self.init_UI_state["controllerPositionState"]["rightController"][u'controller\u200bRotation']
                # turn it into a left handed rotation vector
                right_handed_init_quat = np.array([-init_quat[2],init_quat[0],init_quat[1],init_quat[3]])
                #transform it to right handed:

                curr_quat = self.UI_state["controllerPositionState"]["rightController"][u'controller\u200bRotation']

                right_handed_curr_quat = np.array([-curr_quat[2],curr_quat[0],curr_quat[1],curr_quat[3]])
                #transform it to right handed:
                #right_handed_curr_rotvec =  np.array([-left_handed_curr_rotvec[2],left_handed_curr_rotvec[0],-left_handed_curr_rotvec[1]])
                #Print(right_handed_curr_rotvec)
                #RR_cw_ch = R.from_quat(self.init_UI_state["controllerPositionState"]["rightController"]["u'controller\u200bRotation'"])
                RR_cw_ch = R.from_quat(right_handed_init_quat)
                RR_cw_ch_T = R.from_dcm(RR_cw_ch.as_dcm().transpose())
                RR_cw_cc = R.from_quat(right_handed_curr_quat)
                #print(self.init_UI_state["controllerPositionState"]["rightController"]["u'controller\u200bRotation'"],'\n\n\n\n\n\n')

                
                # if self.UI_state["controllerButtonState"]["rightController"]["press"][1] == True :
                #     RR_cw_cc = RR_cw_ch*RR_unit_x
                # elif self.UI_state["controllerButtonState"]["rightController"]["press"][2] == True :
                #     RR_cw_cc = RR_cw_ch*RR_unit_y

                # elif self.UI_state["controllerButtonState"]["rightController"]["press"][3] == True :
                #     RR_cw_cc = RR_cw_ch*RR_unit_z
                # else:
                #     return 
                # print((RR_cw_ch_T*RR_cw_cc).as_rotvec())
                RR_final = (RR_cw_ch_T*RR_cw_cc*RR_rw_rh).as_dcm().flatten().tolist()
                # self.robot.setRightEEInertialTransform([RR_cw_cc.as_dcm().flatten().tolist(),RT_final],0.025)
                self.robot.setRightEEInertialTransform([RR_rw_rh.as_dcm().flatten().tolist(),RT_final],0.025)

        

        except Exception as e: 
            print("error during execution of set position\n\n\n\n")
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
    
def on_message(ws, message):
    global userId,roomId,drone,roomname, zonename
    global is_closed
    # print "Received ::::::: '%s'" % message
    mjson = json.loads(message)
    if mjson["a"]==0:
        a= {"a":1,"c":0,"p":{"zn": zonename,"un":"","pw":""}}
        b= json.dumps(a).encode('utf-8')
        ws.send(b)

    if mjson["a"]==1:
        if 'id' in mjson["p"]:
            # print "userId: %d" % mjson["p"]["id"]
            userId = mjson["p"]["id"]
            a={"a":4,"c":0,"p":{"n":roomname}}
            b= json.dumps(a).encode('utf-8')
            ws.send(b)

    if mjson["a"]==1001: #Once after room jon sending welcome message
        if roomId ==-1:
            roomId = mjson["p"]["r"]
            # print "Room id :::: '%s'" % roomId

        robotrec={"title": "Robot Telemetry Data","status": {"healthy": "True","eStop": "False","softwareEStop": "False"},"currentConfig": {"leftArm": [-0.20578666666666667,-2.1202733333333335,-1.6030666666666669,3.7186333333333335,-0.96508,0.0974],"rightArm": [0.2028,-1.035932653589793,1.6057333333333335,-0.5738406797435401,0.9622,-0.0974],"torso": [0.1,1.1],"baseOdometry": [0.1,0.2,0.4]},"currentVelocity": {"leftArm": [0,0,0,0,0,0],"rightArm": [0,0,0,0,0,0],"baseWheels": [1.1,0.2],"base": [-0.5,0.5,0]},"targetConfig": {"leftArm": [0.1,1.1,2.1,3.1,4.1,5.1],"rightArm": [0.1,1.1,2.1,3.1,4.1,5.1],"torso": [0.1,1.1],"baseOdometry": [0.1,0.2,0.4]},"controller": {"collisionWarning": {"leftArm": "True","rightArm": "False"},"unreachableWarning": {"leftArm": "True","rightArm": "False"}},"perception": {"miniMap": {"width": 2,"height": 1,"matrix": [[0],[0]]},"hapticFeedback": [0,1,2]}}
        
	   
        #sending public messages  
        a={"a":7,"c":0,"p":{"t":0,"r":roomId,"u":userId,"m":"robot_telemetry","p":robotrec}}
        b= json.dumps(a).encode('utf-8')
        ws.send(b)

                                        
    if mjson["a"]==7:  #Retrieve public message
        if mjson["p"]["m"]=="controllers" and mjson["p"]["r"]==roomId:
            robotrec={"title": "Robot Telemetry Data","status": {"healthy": "True","eStop": "False","softwareEStop": "False"},"currentConfig": {"leftArm": [-0.20578666666666667,-2.1202733333333335,-1.6030666666666669,3.7186333333333335,-0.96508,0.0974],"rightArm": [0.2028,-1.035932653589793,1.6057333333333335,-0.5738406797435401,0.9622,-0.0974],"torso": [0.1,1.1],"baseOdometry": [0.1,0.2,0.4]},"currentVelocity": {"leftArm": [0,0,0,0,0,0],"rightArm": [0,0,0,0,0,0],"baseWheels": [1.1,0.2],"base": [-0.5,0.5,0]},"targetConfig": {"leftArm": [0.1,1.1,2.1,3.1,4.1,5.1],"rightArm": [0.1,1.1,2.1,3.1,4.1,5.1],"torso": [0.1,1.1],"baseOdometry": [0.1,0.2,0.4]},"controller": {"collisionWarning": {"leftArm": "True","rightArm": "False"},"unreachableWarning": {"leftArm": "True","rightArm": "False"}},"perception": {"miniMap": {"width": 2,"height": 1,"matrix": [[0],[0]]},"hapticFeedback": [0,1,2]}}
            robot_telemetry = my_controller._serveStateReciever(mjson["p"]["p"])
            # robotrec["currentConfig"]["leftArm"] = robot_telemetry[0]*math.pi/180
            # # robotrec["currentConfig"]["rightArm"] = robot_telemetry[2]
            # # robotrec["currentvelocity"]["leftArm"] = robot_telemetry[1]
            # # robotrec["currentConfig"]["rightArm"] = robot_telemetry[3]
            # robotrec["currentConfig"]["rightArm"] = robot_telemetry[1]*math.pi

            # a={"a":7,"c":0,"p":{"t":0,"r":roomId,"u":userId,"m":"robot_telemetry","p":robotrec}}
            # b= json.dumps(a).encode('utf-8')
            # ws.send(b)
        # print "=================="
        # print "Controller Sent   "
        # print "=================="
        
        # print mjson["p"]["p"]   
        # print "==================="
        #     print "leftController press " ,mjson["p"]["p"]["controllerButtonState"]["leftController"]["press"]


def on_error(ws, error):
    # print(error)
    pass

def on_close(ws):
    global is_closed
    is_closed=1
    print("### closed ###")


def on_open(ws):
    def run(*args):
        global is_closed
        a = {"a":0,"c":0,"p":{"api":"1.2.0","cl":"JavaScript"}}
        b =json.dumps(a).encode('utf-8')
        # print(a)
        ws.send(b)

        while is_closed==0:
            time.sleep(1)
        time.sleep(1)
        ws.close()
        # print("Thread terminating...")

    Thread(target=run).start()








       

if __name__ == "__main__" : 
    my_controller = UIController()
    websocket.enableTrace(True)
    host = "ws://gametest.vrotors.com:8888/websocket"
    ws = websocket.WebSocketApp(host,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()