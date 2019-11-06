import time,math
from klampt import vis
from klampt import WorldModel
import threading
from Motion.motion_client import MotionClient
from Motion.motion import Motion
import json

model_name = "Motion/data/TRINA_world_reflex.xml"

class UIRobotClient:
    def __init__(self):
        self.dt = 0.02
        self.robot = MotionClient()
        
        res = self.robot.startup()
        if not res:
            return
        world = WorldModel()
        res = world.readFile(model_name)
        if not res:
            raise RuntimeError("Unable to load Klamp't model")
       
        self.vis_robot = world.robot(0)

        controlThread = threading.Thread(target = self._visualUpdateLoop)
        controlThread.start()

        vis.add("world",world)
        vis.show()

    def _visualUpdateLoop(self):
        while not self.robot.isShutDown:
            q = self.robot.getKlamptSensedPosition()
            self.vis_robot.setConfig(q)
            time.sleep(self.dt)
            print(self.dt)

    def moveRobotTest(self):
        self.robot.setBaseVelocity([1,1])

if __name__ == "__main__" : 
    a = UIRobotClient()
    a.moveRobotTest()