import klampt
import time
import math
from klampt import vis
from klampt import io
from klampt.math import so2
from klampt.model import sensing
import matplotlib.pyplot as plt

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
        pt[0] += 0.2    # lidar is 0.2 offset (in x direction) from robot
        pt = so2.apply(theta, pt)
        pt[0] += x
        pt[1] += y
        pt.append(0.2)  # height is 0.2

        rv.addPoint(pt)

    return rv

robot = Motion(mode = "Kinematic", codename="anthrax")

leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)

robot.startup()

# reset arms
robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
robot.setRightLimbPositionLinear(rightUntuckedConfig,5)

world = robot.getWorld()

vis.add("world", world)
vis.show()

sim = klampt.Simulator(world)

lidar = sim.controller(0).sensor("lidar")
left_cam = sim.controller(0).sensor("left_hand_camera")
right_cam = sim.controller(0).sensor("right_hand_camera")
vis.add("lidar", lidar)
vis.add("left_cam", left_cam)
#vis.add("right_cam", right_cam)

#time.sleep(3)

start_time = time.time()
while True:
    lidar.kinematicSimulate(world, 0.01)
    left_cam.kinematicSimulate(world, 0.01)
    #right_cam.kinematicSimulate(world, 0.01)

    measurements = lidar.getMeasurements()
    lidar_pc = lidar_to_pc(robot, lidar, measurements)
    vis.add("pc", lidar_pc)

    # rgb image, depth_image
    lc_rgb, lc_depth = sensing.camera_to_images(left_cam)

    # Get point cloud. For some reason triggers an assertion for me in Python2
    # with the latest version on Klampt Master
    left_cam_pc = sensing.camera_to_points(left_cam, points_format='Geometry3D', all_points=True, color_format='rgb')
    left_cam_pc.saveFile("/home/avatrina-gpu/rohan/gpd/gpd/build/temp.pcd")
    
    if time.time() - start_time < 5:
        robot.setBaseVelocity([0.3, 0.15])
    else:
        robot.setBaseVelocity([0.0,0.5])
    time.sleep(0.01)
