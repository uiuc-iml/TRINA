import klampt
import time
import math
from klampt import vis
from klampt.math import so2

from motion import *

def interpolate(a, b, u):
    return (b-a)*u + a

def lidar_to_pc(robot, sensor, scan):
    x, y, theta = robot.base_state.measuredPos
    angle_range = float(sensor.getSetting("xSweepMagnitude"))

    rv = klampt.PointCloud()

    for i in range(len(scan)):
        u = float(i)/float(len(scan))
        # If period = 0, measurement sweeps over range of [-magnitude,magnitude]
        angle = interpolate(-angle_range, angle_range, u)
        pt = [math.cos(angle), math.sin(angle)]
        pt[0] *= scan[i]
        pt[1] *= scan[i]
        pt = so2.apply(theta, pt)
        pt[0] += x
        pt[1] += y
        pt.append(0.2)

        rv.addPoint(pt)

    return rv

robot = Motion(mode = "Kinematic", codename="seed")
robot.startup()

world = robot.getWorld()

vis.add("world", world)
vis.show()

sim = klampt.Simulator(world)

lidar = sim.controller(0).sensor("lidar")
vis.add("lidar", lidar)

time.sleep(3)

start_time = time.time()

while True:
    vis.lock()
    lidar.kinematicSimulate(world, 0.01)

    measurements = lidar.getMeasurements()
    pc = lidar_to_pc(robot, lidar, measurements)

    vis.add("pc", pc)

    if time.time() - start_time < 5:
        robot.setBaseVelocity([0.5, 0.15])
    else:
        robot.setBaseVelocity([0.0,0.0])
    vis.unlock()
    time.sleep(0.01)
