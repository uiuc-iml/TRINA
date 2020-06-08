import rospy
import math
import tf
import klampt
from klampt import Geometry3D
from klampt.io import ros
from klampt.math import so2

import numpy as np

from nav_msgs.srv import GetMap

def get_occupancy_grid(topic = "dynamic_map", timeout = None):
    try:
        rospy.wait_for_service(topic, timeout = timeout)
    except rospy.ROSException:
        return None
    try:
        srv = rospy.ServiceProxy(topic, GetMap)
        resp1 = srv()
        return resp1.map
    except rospy.ServiceException as e:
        pass
        #print("Service call failed: {}".format(e))

def build_2d_map(occupancy_grid):
    width, height = occupancy_grid.info.width, occupancy_grid.info.height

    closest_obstacle = {}

    rv = np.zeros((height, width))
    for i in range(height):
        for j in range(width):

            if not (i, j) in closest_obstacle:
                closest_obstacle[(i, j)] = []

            if rv[i, j] != 0:
                continue

            val = occupancy_grid.data[i*width + j]

            # if unknown, assume it is open
            if val == 0 or val == -1:
                rv[i, j] = 0
            else:
                rv[i, j] = val

    return rv

def l2_dist(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1]-b[1])**2)

def clip(mini, maxi, val):
    if val < mini:
        return mini
    elif val > maxi:
        return maxi
    return val

def transform_coordinates(point, occupancy_grid):
    # world -> grid
    try:
        x, y = point
        theta = None
    except Exception:
        x, y, theta = point

    resolution = occupancy_grid.info.resolution

    x_origin, y_origin = occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y

    #x_new = (x - x_origin) / resolution
    #y_new = (y - y_origin) / resolution
    x_new = x / resolution
    y_new = y / resolution

    x_new += -x_origin + occupancy_grid.info.width/2
    y_new += -y_origin + occupancy_grid.info.height/2

    if theta is None:
        return x_new, y_new
    else:
        return x_new, y_new, theta

def transform_back(point, occupancy_grid):
    x, y = point
    resolution = occupancy_grid.info.resolution

    x_origin, y_origin = occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y

    #x_new = x * resolution
    #y_new = y * resolution
    #x_new += x_origin
    #y_new += y_origin

    x_new = x - (-x_origin + occupancy_grid.info.width/2)
    y_new = y - (-y_origin + occupancy_grid.info.height/2)
    x_new *= resolution
    y_new *= resolution

    return [x_new, y_new]

def intify(point):
    return (int(point[0]), int(point[1]))

def publish_tf(curr_pose):
    x, y, theta = curr_pose
    theta = theta % (math.pi*2)
    br = tf.TransformBroadcaster()
    br.sendTransform([-x, -y, 0], tf.transformations.quaternion_from_euler(0, 0, -theta), rospy.Time.now(), "odom", "base_link")
    br.sendTransform([0.2, 0, 0.2], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_link", "base_scan")

def publish_tf(curr_pose):
    x, y, theta = curr_pose
    theta = theta % (math.pi*2)
    br = tf.TransformBroadcaster()
    br.sendTransform([-x, -y, 0], tf.transformations.quaternion_from_euler(0, 0, -theta), rospy.Time.now(), "odom", "base_link")
    br.sendTransform([0.2, 0, 0.2], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_link", "base_scan")

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

def add_terrain(world, path, T, name):
    geom = Geometry3D()
    geom.loadFile(path)
    geom.transform(T[0], T[1])
    item = world.makeTerrain(name)
    item.geometry().set(geom)
    return item

def interpolate(a, b, u):
    return (b-a)*u + a
