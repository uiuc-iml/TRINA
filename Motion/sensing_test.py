import klampt
import time
from klampt import vis

from motion import *

robot = Motion(mode = "Kinematic")
robot.startup()

world = robot.getWorld()

vis.add("world", world)
vis.show()

sim = klampt.Simulator(world)

sensor = sim.controller(0).sensor("lidar")
vis.add("sensor", sensor)
#print(sensor.getSetting("link"))

while True:
    vis.lock()
    sensor.kinematicSimulate(world, 0.01)
    #sim.simulate(0.01)
    sim.updateWorld()
    print(sensor.getMeasurements())
    vis.unlock()
    time.sleep(0.01)
