import matplotlib.pyplot as plt

from global_planner import *
from local_planner import *
from motion_primitives import *
from utils import *
from geometry import *

if __name__ == "__main__":

    radius = 10

    grid = get_occupancy_grid()
    gridmap = build_2d_map(grid)

    start_point = (0, 0)
    #start_point = (100, 270)
    end_point = (450, 200) # (y, x)
    #end_point = (100, 270)
    #end_point = (100, 400)
    #end_point = (350, 400)
    #end_point = (450, 500)

    dists, parents = navigation_function(gridmap, end_point, radius)

    global_path = get_path(parents, start_point, end_point)
    if global_path is None:
        raise Exception("No global path found!")

    curr_point = Circle(start_point[::-1], radius)
    curr_theta = 0

    while True:
        plt.cla()
        #plt.imshow(dists, cmap="hot_r", interpolation="none")
        plt.imshow(dists)
        plt.plot(global_path.get_xs(), global_path.get_ys())
        curr_point.plot(plt)

        if curr_point.collides(gridmap):
            print("collided...this should not have happened")
            continue

        dist_to_goal = l2_dist(curr_point.center, end_point[::-1])
        if dist_to_goal < 50:
            primitives = [LocalPath([(curr_point.center[0], curr_point.center[1], curr_theta), (end_point[1], end_point[0], curr_theta)])]
        elif dist_to_goal < 5:
            continue
        else:
            primitives = get_primitives(curr_point.center, curr_theta, 20, 20)

        closest = evaluate_primitives(curr_point, primitives, global_path, gridmap)

        for prim in primitives:
            c = 'r'
            if prim == closest:
                c = 'g'
            xs, ys, thetas = prim.get_xytheta(200)
            plt.plot(xs, ys, c)

        if closest is None:
            print("No valid prim... :(")
            continue

        xs, ys, thetas = closest.get_xytheta(200)
        next_point = (xs[-10], ys[-10])
        new_theta = thetas[-10]

        curr_point = Circle(next_point, radius)
        curr_theta = new_theta

        plt.pause(0.01)
        plt.draw()
