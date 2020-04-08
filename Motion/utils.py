import rospy
import math
import numpy as np

from nav_msgs.srv import GetMap

def get_occupancy_grid():
    rospy.wait_for_service("static_map")
    try:
        srv = rospy.ServiceProxy("static_map", GetMap)
        resp1 = srv()
        return resp1.map
    except rospy.ServiceException, e:
        print("Service call failed: %s".format(e))

def build_2d_map(occupancy_grid):
    width, height = occupancy_grid.info.width, occupancy_grid.info.height

    rv = np.zeros((height, width))
    for i in range(height):
        for j in range(width):
            if rv[i, j] != 0:
                continue

            val = occupancy_grid.data[i*width + j]
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
