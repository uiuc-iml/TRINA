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

from trina import jarvis
from trina.modules.UI import UIAPI
import weakref

class RemoteUI(jarvis.Module):

    def __init__(self,Jarvis):
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
        jarvis.Module.__init__(self,Jarvis)
        self.ws_server = SimpleWebSocketServer('', 8000, lambda *args:UIStateReciever(self,*args))
        self.startCustomThread(self._ws_server_loop)

    @classmethod
    def name(self):
        return "UI"

    @classmethod
    def apiClass(self):
        return UIAPI

    def _ws_server_loop(self):
        self.ws_server.serveforever()

    def terminate(self):
        self.ws_server.close()
        jarvis.Module.terminate()


class UIStateReciever(WebSocket) : 
    """the websocket server implementation for recieving UI JSON state from the UI 
        
    """
    def __init__(self,module,*args):
        WebSocket.__init__(self,*args)
        self.module = weakref.proxy(module)
        self.state_server = module.jarvis.server
    def handleMessage(self) : 
        try:
            obj = json.loads(self.data)
            title = obj["title"]
        except Exception as err:
            print("Error: {0}".format(err))
            return

        if title == "UI Outputs":
            self.UI_state = obj
            self.state_server["UI_STATE"] = self.UI_state

        elif title == "UI API FUNCTION FEEDBACK":
            id = obj['id']
            msg = obj['MSG']
            self.module.setRedisRpc(id,msg)

        try:
            command = self.module.getRedisRpc()
            if command:
                if command['from'] == self.module.name():
                    print("ignoring command from " + command['from'] + "command recieved was" + command['fn'])
                else:
                    message = {'title':"UI API FUNCTION CALL", 'id':command['id'], 'funcName':command['fn'],  'args':command['args']}
                    print("pass")
                    self.sendMessage(json.dumps(message))
        except Exception as err:
                print("Error: {0}".format(err))

        print("message from UI")

    def handleConnected(self) : 
        print(self.address, 'connected')

    def handleClose(self) : 
        print(self.address, 'closed')




if __name__ == "__main__":
    jv = jarvis.Jarvis('UI',['Motion'])
    module = RemoteUI(jv)
    while True:
        time.sleep(1.0)
    

       