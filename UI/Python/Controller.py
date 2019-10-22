import pprint
import json
from Motion.motion import Motion
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
import time,math
from klampt import vis,ik

robot_ip = '130.126.139.236'
ws_port = 1234
robot_mode = 'Kinematic'


class UIController(WebSocket) : 
    # UI state object detailing controller and headset states/position as well as UI logic
    UI_state = {}
    # robot controller api porvided by control team
    robot = Motion(mode = robot_mode)

    def handleMessage(self) : 
        print("message from UI:"  + self.data)
        obj = json.loads(self.data)

        if not obj["title"] == "UI Outputs" :
           return

        self.UI_state = obj
        self.checkState()

    def handleConnected(self) : 
        print(self.address, 'connected')


    def handleClose(self) : 
        print(self.address, 'closed')

    def checkState(self) :
        if not self.UI_state["title"] == "UI Outputs" :
            print("UI state object INVALID")
            return
        #different cases 
        if self.UI_state["controllerPositionState"]["leftController"]["controllerPosition"][1] > 0 :
            print("case 1")
            self.robotDemoTest()
            
    def robotDemoTest(self):
        self.robot.startup()
        print('Robot start() called')
        startTime = time.time()
        world = self.robot.getWorld()
        vis.add("world",world)
        vis.show()
        while (time.time()-startTime < 5):
            vis.lock()
            self.robot.setBaseVelocity([0.5,0.1])
            vis.unlock()
            time.sleep(0.02)
            print(time.time()-startTime)
        self.robot.shutdown()



if __name__ == "__main__" : 
    server = SimpleWebSocketServer(robot_ip, ws_port, UIController)
    server.serveforever()
