# from UI_end_1 import UI_end_1
# from rpc_queue import rpc_queue
import time,math
import uuid 
import threading
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import klampt
from klampt import io
from klampt.model import ik,coordinates,config,trajectory,collide
import json
from SimpleWebSocketServer import SimpleWebSocketServer,WebSocket


class UI:

    def __init__(self,simulation_level = 1):
        """
        This class serves to provide communications between the internal TRINA system and the
UI end. Such communications provide TRINA app developers UI related commands such as add
texts, send trajectories while also report UI operator input to the internal TRINA system such as
key presses and headsets orientations.
        --------
        simulation_level: the UI End specified to visualize TRINA command and sends UI Input:
            UI END 1: This is a simplified UI end that visualize all the UI commands. It tries to decouple
                    from all the connections and data transfer issues. Thus, providing the closet level of debugging
                    experience.
            UI END 2: This is placed at the same location as the actual UI End in the final product. It
                    accepts the same UI calls and reports the same UI state from and to TRINA just as the final
                    UI END, the only difference is UI END 2 uses Klampt as the UI visualization technology.
            UI END 3: This part will be developed by VRotors. A toy example via Unity might be built
                    for reference for the scope of this project.
        """
        self.simulation_level = simulation_level

        self.processes = []


    def shutdown(self):
        print('shutting down UI')
        for i in self.processes:
            i.terminate()

    def return_processes(self):
        return self.processes


class UIStateReciever(WebSocket): 
    """the websocket server implementation for recieving UI JSON state from the UI 
        
    """
    def handleMessage(self): 
        try:
            self.commandQueue = self.server["UI_END_COMMAND"].read()
            if  self.commandQueue:
                command = self.commandQueue[0]
                self.commandQueue.pop(0)
                self.server["UI_END_COMMAND"] = self.commandQueue
                if command['from'] != self.mode:
                    print("ignoring command from " + command['from'] + "command recieved was" + command['funcName'])
                else:
                    message = {'title':"UI API FUNCTION CALL", 'id':command['args']['id'], 'funcName':command['funcName'],  'args':command['args']}
                    print("pass")
                    self.sendMessage(json.dumps(message))
        except Exception as err:
                print("Error: {0}".format(err))

        print("message from UI")

        try:
            obj = json.loads(self.data)
            title = obj["title"]
        except Exception as err:
            print("Error: {0}".format(err))
            return

        if title == "UI Outputs":
            self.UI_state = obj
            self.server["UI_STATE"] = self.UI_state

        elif title == "UI API FUNCTION FEEDBACK":
            id = obj['id']
            msg = obj['MSG']
            self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':True, 'MSG':msg}

        elif title == "Phone Estop":
            stop = obj["UIlogicState"]["stop"]
            self.server["Phone_Stop"] = stop
            
            


        

    def handleConnected(self): 
        print(self.address, 'connected')
        self.mode = "PointClickNav"
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["Phone_Stop"] = False
        self.server["VR_Stop"] = False


    def handleClose(self): 
        print(self.address, 'closed')


   


if __name__ == "__main__":
    server = SimpleWebSocketServer('', 8000, UIStateReciever)
    server.serveforever()


       