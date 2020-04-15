from motion import *
from klampt.math import vectorops,so2
from klampt import vis
from klampt import Geometry3D
import klampt
import math
import time
import matplotlib.pyplot as plt

from global_planner import *
from local_planner import *
from motion_primitives import *
from motion_profile import *
from utils import *
from geometry import *

##### testing using the kinematic mode..############
#
###################################################
robot = Motion(mode = 'Kinematic', codename="seed")

world = robot.getWorld()

radius = 0.5588/2

vis.add("world",world)

start = (0, 0)
end = (4, 8)

grid = get_occupancy_grid()
gridmap = build_2d_map(grid)

dists, parents = navigation_function(gridmap, end, radius)
global_path = get_path(parents, start, end)
print(global_path.points)
if global_path is None:
    raise Exception("No global path found!")

curr_point = Circle(start[::-1], radius)
curr_theta = 0
end_theta = math.pi/2

robot.startup()
print('Robot start() called')
vis.show()

end_v = 0.5

while True:

    if curr_point.collides(gridmap):
        print("collided...this should not have happened")
        continue

    dist_to_goal = l2_dist(curr_point.center, end)
    if dist_to_goal < 0.2:
        break

    # near goal, run a special controller?
    if dist_to_goal < 1.0:
        primitives = [LocalPath([(curr_point.center[0], curr_point.center[1], curr_theta), (end[0], end[1], end_theta)])]
        end_v = 0
    else:
        primitives = get_primitives(curr_point.center, curr_theta, radius*4, radius*2)

    closest = evaluate_primitives(curr_point, primitives, global_path, gridmap)

    kglobal = klampt.model.trajectory.Trajectory(milestones = [[x, y] for x, y in zip(global_path.get_xs(), global_path.get_ys())])
    vis.add("kglobaltraj", kglobal)

    if closest is None:
        print("No prim!!!")
        continue

    xs, ys, thetas = closest.get_xytheta(200)

    plt.cla()
    plt.plot(global_path.get_xs(), global_path.get_ys())
    for p in primitives:
        px, py, _ = p.get_xytheta(200)
        plt.plot(px, py, color='r')
    plt.plot(xs, ys, color='g')
    curr_point.plot(plt)
    plt.pause(0.01)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.draw()

    acc = 0.5
    max_v = 0.5
    profile = generate_profile(closest, acc, max_v, 0.01, end_v = end_v, start_v=robot.base_state.measuredVel[0])
    end_v = profile[-1].v
    ktraj = klampt.model.trajectory.Trajectory(milestones = [[x, y] for x, y in zip(xs, ys)])

    N = len(profile)
    time_thresh = 0.2
    start_time = time.time()

    for i in range(N):
        iteration_start = time.time()
        state = profile[i]

        if iteration_start - start_time > time_thresh:
            end_v = state.v
            break

        pose = robot.base_state.measuredPos

        curr_target = (state.x, state.y)

        trans_target = so2.apply(-pose[2], vectorops.sub(curr_target, (pose[0], pose[1])))

        linear_error = trans_target[0]
        cross_track_error = trans_target[1]

        k_linear = 1.0
        k_angular = 1.0

        vis.lock()
        vis.add("klocaltraj", ktraj)
        vis.setColor("klocaltraj", 0, 0, 255)
        robot.setBaseVelocity([state.v + k_linear*linear_error, state.w + k_angular*cross_track_error])
        vis.unlock()

        new_pose = robot.base_state.measuredPos
        curr_theta = new_pose[2]

        elapsed_time = time.time() - iteration_start
        remaining_time = max(0.01 - elapsed_time, 0.0)
        time.sleep(remaining_time)

    new_pose = robot.base_state.measuredPos
    curr_point = Circle((new_pose[0], new_pose[1]), radius)
    curr_theta = new_pose[2]
    print(curr_point.center)
    print("Distance to goal:", l2_dist(curr_point.center, end))

robot.setBaseVelocity([0, 0])
while True:
    time.sleep(0.5)

vis.kill()
robot.shutdown()
