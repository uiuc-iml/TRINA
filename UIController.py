import pprint
import json
from Motion.motion import Motion
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
import time,math
from klampt import vis


# ws://localhost:9000
robot_ip = '130.126.139.236'
ws_port = 1234
robot_mode = 'Kinematic'
clients = {}

class UIController(WebSocket) : 
    # UI state object detailing controller and headset states/position as well as UI logic
    UI_state = {}
    # robot controller api porvided by control team
    robot = Motion(mode = robot_mode)
    world = robot.getWorld()
    robot.startup()
    vis.show()
    vis.add("world",world)

    def handleMessage(self) : 
        print("message from UI")
        # print(self.data)
        obj = json.loads(self.data)
        # print(obj)
        if not obj["title"] == "UI Outputs" :
            print(self.data)
            return

        self.UI_state = obj
        try:
            self.printState()
        except:
            print("-----")
        self.checkState()

    def handleConnected(self) : 
        print(self.address, 'connected')


    def handleClose(self) : 
        print(self.address, 'closed')

    def checkState(self):
        print("checking!")
        if not self.UI_state["title"] == "UI Outputs" :
            print("UI state object INVALID")
            return
        #different cases 

        base_velocity = [0.1*(-self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"][1]),0.1*(self.UI_state["controllerButtonState"]["leftController"]["thumbstickMovement"][0])]
        self.robot.setBaseVelocity(base_velocity)
       

    def printState(self):
        print "---------------------------------------UI STATE--------------------------------"
        print "controllerButtonState:"
        print "---leftController:"
        print "------------press:", 
        print  self.UI_state["controllerButtonState"]["leftController"]["press"]
        print "------------touch:" , 
        print  self.UI_state["controllerButtonState"]["leftController"]["touch"]
        print "------------nearTouch:" , 
        print  self.UI_state["controllerButtonState"]["leftController"]["nearTouch"]
        print "------------squeeze:" , 
        print  self.UI_state["controllerButtonState"]["leftController"]["squeeze"]
        print "------------thumbstickMovement:" , 
        print  self.UI_state["controllerButtonState"]["leftController"]["thumbstickMovement"]
        print "---rightController:"
        print "------------press:" , 
        print  self.UI_state["controllerButtonState"]["rightController"]["press"]
        print "------------touch:" , 
        print  self.UI_state["controllerButtonState"]["rightController"]["touch"]
        print "------------nearTouch:" , 
        print  self.UI_state["controllerButtonState"]["rightController"]["nearTouch"]
        print "------------squeeze:" , 
        print  self.UI_state["controllerButtonState"]["rightController"]["squeeze"]
        print "------------thumbstickMovement:" , 
        print  self.UI_state["controllerButtonState"]["rightController"]["thumbstickMovement"]
        print "headSetPositionState:"
        print "------------deviceRotation:" , 
        print  self.UI_state["headSetPositionState"]["deviceRotation"]
        print "controllerPositionState:"
        print "---leftController:"
        print "------------controllerOrientation:" , 
        print  self.UI_state["controllerPositionState"]["leftController"]["controllerOrientation"]
        print "------------controllerPosition:" , 
        print  self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"]
        print "---rightController:"
        print "------------controllerOrientation:" , 
        print  self.UI_state["controllerPositionState"]["rightController"]["controllerOrientation"]
        print "------------controllerPosition:" , 
        print self.UI_state["controllerPositionState"]["rightController"]["controllerPosition"]
        print "UIlogicState:"
        print "------------stop:",
        print self.UI_state["UIlogicState"]["stop"]
        print "------------teleoperationMode:",
        print self.UI_state["UIlogicState"]["teleoperationMode"]
        print "------------autonomousMode:",
        print self.UI_state["UIlogicState"]["autonomousMode"]
        print "-----------------------------------end-------------------------------------"



if __name__ == "__main__" : 

    server = SimpleWebSocketServer(robot_ip, ws_port, UIController)
    server.serveforever()
   
