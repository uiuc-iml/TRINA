import matplotlib.pyplot as plt

from global_planner import *
from local_planner import *
from motion_primitives import *
from utils import *

if __name__ == "__main__":
    grid = get_occupancy_grid()
    gridmap = build_2d_map(grid)

    start_points = [(0, 0), (100, 270)]

    end_point = (450, 200) # (y, x)
    #end_point = (100, 270)
    #end_point = (100, 400)
    #end_point = (350, 400)
    #end_point = (450, 500)

    dists, parents = navigation_function(gridmap, end_point)

    for p in start_points:
        global_path = get_path(parents, p, end_point)
        if global_path is None:
            print("No global path found for {}!".format(p))
            continue
        plt.plot(global_path.get_xs(), global_path.get_ys())

    plt.imshow(dists, cmap="hot_r", interpolation="none")
    plt.show()
