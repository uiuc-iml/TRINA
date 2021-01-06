import numpy as np
from .local_planner import *
from .utils import *
import math
from .geometry import *

import matplotlib.pyplot as plt

def test_prims():
    primitives = get_primitives((0., 0.), 0., 1., 1.)
    p = primitives[5]
    N = 25
    px, py, _ = p.get_xytheta(N)
    curvature = []
    indexes = []
    for i in range(N):
        curvature.append(p.get_curvature(float(i)/float(N)))
        indexes.append(i)

    plt.scatter(px[:10], py[:10], c="blue")
    plt.scatter(px[10:], py[10:], c="red")
    plt.figure()
    plt.scatter(indexes, curvature)
    plt.show()

def test_prims_2():
    radius = 0.5588/2.
    curr_point = (0.0, 0.0)
    curr_theta = 0.0

    robot = Circle(center = curr_point, radius = radius)
    primitives = get_primitives(curr_point, curr_theta, radius*2, radius)
    for p in primitives:
        xs, ys, _ = p.get_xytheta(200)
        plt.plot(xs, ys, color="red")

    #best = evaluate_primitives(robot, primitives, global_path, gridmap)
    plt.show()

def get_primitives(curr_point, curr_theta, dx, dy):
    curr_pose = (curr_point[0], curr_point[1], curr_theta)

    prims = [
        [dx, 0, 0],

        [dx, dy/2, 0],
        [dx, -dy/2, 0],

        [dx, dy/2, math.pi/8],
        [dx, -dy/2, -math.pi/8],

        [dx, dy/4, 0],
        [dx, -dy/4, 0],

        [dx, dy/4, math.pi/8],
        [dx, -dy/4, -math.pi/8],

        [dx/2, dy/2, math.pi/4],
        [dx/2, -dy/2, -math.pi/4],

        [dx/4, dy, math.pi/2],
        [dx/4, -dy, -math.pi/2],
    ]

    rv = []
    for prim in prims:
        dx, dy, dtheta = prim
        target = (curr_pose[0] + dx*math.cos(curr_theta) - dy*math.sin(curr_theta), curr_pose[1] +dx*math.sin(curr_theta) + dy*math.cos(curr_theta), (curr_theta + dtheta)%(2*math.pi))
        path = LocalPath([curr_pose, target])
        rv.append(path)

    return rv

def forward_simulate(robot, prim, gridmap):
    N = 50
    xs, ys, thetas = prim.get_xytheta(N)
    height, width = gridmap.shape

    for i in range(N):
        curr_x = xs[i]
        curr_y = ys[i]

        curr_pose = Circle((curr_x, curr_y), robot.radius)
        if curr_pose.collides(gridmap):
            return False

    return True

def evaluate_primitives(robot, primitives, global_path, gridmap):
    if len(primitives) == 1:
        return primitives[0]

    closest_score = 1e9
    closest_prim = None

    for prim in primitives:

        if prim.collides(gridmap):
            continue

        if not forward_simulate(robot, prim, gridmap):
            continue

        xs, ys, thetas = prim.get_xytheta(200)
        end_point = (xs[-1], ys[-1])
        end_theta = thetas[-1]

        dist = global_path.get_distance(end_point)

        closest_angle = global_path.get_closest_angle(end_point)
        angle_delta = abs(closest_angle - end_theta)

        dist_remaining = global_path.distance_remaining(end_point)

        score = dist + 0.5*angle_delta + 0.5*dist_remaining
        #print(score, dist, angle_delta, dist_remaining)

        # minimize the sum of (distance error, angle error, and distance remaining)
        if score < closest_score:
            #print("best picked!")
            closest_score = score
            closest_prim = prim

    return closest_prim

if __name__ == "__main__":
    test_prims_2()
