# from UI_end_1 import UI_end_1
# from rpc_queue import rpc_queue
import time,math
import uuid 
import threading
from threading import Thread
import json
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
import sys
from Motion.motion_client_python3 import MotionClient
DEGREE_2_RADIAN = 2.0*math.pi/180.0

class UIStateReciever(WebSocket) : 
    """the websocket server implementation for recieving UI JSON state from the UI 
        
    """


    def headControl(self,orientation):
        def mod180(x):
            while x >= 180:
                x = x - 360
            while x < -180:
                x = x + 360
            return x

        def limitTo(x,min,max):
            if x > max:
                return max
            elif x < min:
                return min
            return x

        panAngle = limitTo((self.panLimits["center"] - mod180(orientation["y"] - self.init_headset_rotation["y"])), self.panLimits["min"], self.panLimits["max"])
        tiltAngle = limitTo((self.tiltLimits["center"] + mod180(orientation["x"] - self.init_headset_rotation["x"])), self.tiltLimits["min"], self.tiltLimits["max"])
        print([panAngle*DEGREE_2_RADIAN, tiltAngle*DEGREE_2_RADIAN])

        try:
            self.motion.setHeadPosition([panAngle*DEGREE_2_RADIAN, tiltAngle*DEGREE_2_RADIAN])
        except Exception as err:
            print("Error: {0}".format(err))
            pass



    def handleMessage(self) : 
        # self.servoController.reportServoState()

        print("---------------------------------------------------")

        try:
            obj = json.loads(self.data)
            title = obj["title"]
            if title == "UI Outputs":
                orientation = {"y":obj['headSetPositionState']['deviceRotation'][1],"x":obj['headSetPositionState']['deviceRotation'][0]}
                print(orientation)
                if(self.init_headset_rotation["x"] == 0):
                    self.init_headset_rotation = orientation
                else:
                    self.headControl(orientation)
        except Exception as err:
            print("Error: {0}".format(err))
            return

        

        

    def handleConnected(self) : 
        print(self.address, 'connected')
        self.mode = "PointClickNav"
        try:
            self.motion = MotionClient('http://localhost:8080')
            self.motion.startServer(mode = "Physical", components = ['head'], codename = 'anthrax')
            self.motion.startup()
            self.init_headset_rotation = {"x": 0, "y": 0} #x,y position
            self.panLimits = {"center": 180, "min":90, "max":270} #head limits
            self.tiltLimits = {"center": 180, "min":130, "max":230} #head limits
        except Exception as err:
            print("Error: {0}".format(err))
            return
        


    def handleClose(self) : 
        print(self.address, 'closed')


   


if __name__ == "__main__":
    server = SimpleWebSocketServer('130.126.136.144', 1234, UIStateReciever)
    server.serveforever()