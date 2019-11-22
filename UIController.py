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
        self.init_pos_right = {}
        res = self.robot.startup()
        if not res:
            return
        world = WorldModel()
        res = world.readFile(model_name)
        if not res:
            raise RuntimeError("Unable to load Klamp't model")
        self.setRobotToDefualt()
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
                inputFile = 'UIOutputs.data'
                fd = open(inputFile, 'rb')
                dataset = pickle.load(fd)
                self.UI_state = dataset
                self.UIStateLogic()
            except:
                pass


    def moveRobotTest(self):
        # self.robot.setBaseVelocity([0,0])
        # leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
        # init_leftUntuckedConfig = [0,0,0,0,0,0] #motionAPI format
        # self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
        # self.robot.setBaseTargetPosition([0,0,0],[0.5,0.5])
        # self.robot.setLeftLimbPositionLinear(init_leftUntuckedConfig,5)
        pass

    def setRobotToDefualt(self):
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,1)
        self.robot.setRightLimbPositionLinear(rightUntuckedConfig,1)

    def UIStateLogic(self):

        base_velocity = [0.1*(self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.1*(-self.UI_state["controllerButtonState"]["leftController"]["thumbstickMovement"][0])]
        try:
            # print(" setBaseVelocity successful")
            # print(base_velocity)
            self.robot.setBaseVelocity(base_velocity)
        except:
            print(" setBaseVelocity not successful")
            pass

        try:
            if self.UI_state["controllerButtonState"]["leftController"]["press"][0] == True :
                [leftR,leftT] = self.init_pos_left
                newT = self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
                orgT = self.init_UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
                T = [leftT[0]+(newT[2]-orgT[2]),leftT[1]-(newT[0]-orgT[0]),leftT[2]+(newT[1]-orgT[1])]
                self.robot.setLeftEEInertialTransform([leftR,T],0.02)
        except Exception as e: 
            print(e)
            pass
           
        try:
            if self.UI_state["controllerButtonState"]["rightController"]["press"][0] == True :
                [rightR,rightT] = self.init_pos_right
                newT = self.UI_state["controllerPositionState"]["rightController"]["controllerPosition"]
                orgT = self.init_UI_state["controllerPositionState"]["rightController"]["controllerPosition"]
                T = [rightT[0]+(newT[2]-orgT[2]),rightT[1]-(newT[0]-orgT[0]),rightT[2]+(newT[1]-orgT[1])]
                self.robot.setRightEEInertialTransform([rightR,T],0.02)
        except Exception as e: 
            print(e)
            pass


if __name__ == "__main__" : 
    a = UIController()