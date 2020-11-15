# from UI_end_1 import UI_end_1
# from rpc_queue import rpc_queue
import time,math
import uuid 
import threading
from threading import Thread
import json
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket
import sys
from servoController import servoController

class UIStateReciever(WebSocket) : 
    """the websocket server implementation for recieving UI JSON state from the UI 
        
    """
    def handleMessage(self) : 
        # self.servoController.reportServoState()

        print("---------------------------------------------------")

        try:
            obj = json.loads(self.data)
            title = obj["title"]
            if title == "UI Outputs":
                orientation = {"y":obj['headSetPositionState']['deviceRotation'][1],"x":obj['headSetPositionState']['deviceRotation'][0]}
                print(orientation)
                self.servoController.setGoal(orientation)
        except Exception as err:
            print("Error: {0}".format(err))
            return

        

        

    def handleConnected(self) : 
        print(self.address, 'connected')
        self.mode = "PointClickNav"
        try:
           self.servoController = servoController()
        except Exception as err:
            print("Error: {0}".format(err))
            return
        


    def handleClose(self) : 
        print(self.address, 'closed')


   


if __name__ == "__main__":
    server = SimpleWebSocketServer('130.126.136.144', 1234, UIStateReciever)
    server.serveforever()