from UI_end_1 import UI_end_1
from rpc_queue import rpc_queue
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
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["UI_STATE"] = 0
        self.server["UI_END_COMMAND"] = []

        self.init_UI_state = {}
        self.UI_state = {}
        self.startup = True
        self.dt = 0.025

        stateRecieverThread = threading.Thread(target=self._serveStateReciever)
        stateRecieverThread.start()



    def _serveStateReciever(self):
        while True:
            if self.startup and self.server["UI_STATE"].read()!=0:
                print('started the initial values for the variables')
                # initial ui state for references
                self.init_UI_state = self.server['UI_STATE'].read()
                self.startup = False
            while True:
                try:
                    self.last_time = time.time()
                    self.UI_state = self.modifyRawState(self.server['UI_STATE'].read())
                    time.sleep(self.dt)
                except:
                    pass


    def test(self):
        self._do_rpc({'funcName':'test','args':{}})
        return

    def addText(self, text, position):
        self._do_rpc({'funcName':'addText','args':{'text':text,'position':position}})
        return
    
    def addConfirmation(self,title,text):
        id = uuid.uuid1()
        self._do_rpc({'funcName':'addConfirmation','args':{'id':str(id),'title':title,'text':text}})
        return  id

    def addPrompt(self,title,text):
        id = uuid.uuid1()
        # TODO
        return  id
    
    def addInputBox(self,title,text,fields):
        id = uuid.uuid1()
        # TODO
        return id

    def sendTrajecotry(self,trajectory):
        self._do_rpc({'funcName':'sendTrajecotry','args':{'trajectory':trajectory}})
        return


    def modifyRawState(self, state):
        # placeholder for modifing the ui state when needed
        return state


    def _do_rpc(self,msg):
        commandQueue = self.server["UI_END_COMMAND"].read()
        commandQueue.append(msg)
        self.server["UI_END_COMMAND"] = commandQueue
        print("commandQueue", commandQueue)
        time.sleep(0.0001) 




if __name__ == "__main__":
    a = UI()
    a.test()
    a.addText('testing','text1')

    file_dir = "../../data/TRINA_world_seed.xml"
    world = klampt.WorldModel()
    res = world.readFile(file_dir)
    robot = world.robot(0)
    times = list(range(10))
    milestones = []
    for t in times:
        robot.randomizeConfig()
        milestones.append(robot.getConfig())
    traj = trajectory.RobotTrajectory(robot,times,milestones)
    traj = io.loader.toJson(traj,'Trajectory')
    a.sendTrajecotry(traj)
       