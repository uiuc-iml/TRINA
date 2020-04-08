import numpy as np
import math
from heapq import heappush, heappop

from utils import *

class GlobalPath:

    def __init__(self):
        self.points = []

    def add_point(self, point):
        self.points.append(point)

    def get_xs(self):
        return [p[0] for p in self.points]

    def get_ys(self):
        return [p[1] for p in self.points]

    def distance_remaining(self, point):
        x_closest, y_closest, i = self.get_closest_point_and_idx(point)
        closest_point = (x_closest, y_closest)

        rv = 0
        for i in range(i, len(self.points)-1):
            rv += l2_dist(self.points[i], self.points[i+1])

        rv += l2_dist(self.points[i], closest_point)
        return rv

    def get_closest_point_and_idx(self, point):

        smallest_sq_dist = 1e9
        x_t, y_t = point
        x_t = float(x_t)
        y_t = float(y_t)

        x_closest, y_closest = None, None
        idx_closest = None

        for i in range(1, len(self.points)):
            # make segment
            x1, y1 = self.points[i]
            x2, y2 = self.points[i-1]
            x1 = float(x1)
            y1 = float(y1)
            x2 = float(x2)
            y2 = float(y2)

            px = x2-x1
            py = y2-y1

            # distance along segment
            u = ((x_t-x1)*px + (y_t-y1)*py)/(px * px + py * py)
            if u > 1:
                u = 1
            if u < 0:
                u = 0

            x = x1 + u*px
            y = y1 + u*py

            dx = x-x_t
            dy = y-y_t

            sq_dist = dx*dx + dy*dy

            if sq_dist < smallest_sq_dist:
                smallest_sq_dist = sq_dist
                x_closest = x
                y_closest = y
                idx_closest = i

        return x_closest, y_closest, i

    def get_closest_angle(self, point):
        smallest_sq_dist = 1e9
        angle = None

        x_t, y_t = point
        x_t = float(x_t)
        y_t = float(y_t)

        for i in range(1, len(self.points)):
            # make segment
            x1, y1 = self.points[i]
            x2, y2 = self.points[i-1]
            x1 = float(x1)
            y1 = float(y1)
            x2 = float(x2)
            y2 = float(y2)

            px = x2-x1
            py = y2-y1

            # distance along segment
            u = ((x_t-x1)*px + (y_t-y1)*py)/(px * px + py * py)
            if u > 1:
                u = 1
            if u < 0:
                u = 0

            x = x1 + u*px
            y = y1 + u*py

            dx = x-x_t
            dy = y-y_t

            sq_dist = dx*dx + dy*dy

            if sq_dist < smallest_sq_dist:
                smallest_sq_dist = sq_dist
                angle = math.atan2(y1-y2, x1-x2)

        return angle

    def get_distance(self, point):
        smallest_sq_dist = 1e9
        x_t, y_t = point
        x_t = float(x_t)
        y_t = float(y_t)

        for i in range(1, len(self.points)):
            # make segment
            x1, y1 = self.points[i]
            x2, y2 = self.points[i-1]
            x1 = float(x1)
            y1 = float(y1)
            x2 = float(x2)
            y2 = float(y2)

            px = x2-x1
            py = y2-y1

            # distance along segment
            u = ((x_t-x1)*px + (y_t-y1)*py)/(px * px + py * py)
            if u > 1:
                u = 1
            if u < 0:
                u = 0

            x = x1 + u*px
            y = y1 + u*py

            dx = x-x_t
            dy = y-y_t

            sq_dist = dx*dx + dy*dy

            if sq_dist < smallest_sq_dist:
                smallest_sq_dist = sq_dist

        return smallest_sq_dist

    def sample_points(self, idx, N = 5, max_range = 5):
        point = self.points[idx]
        rv = []
        for i in range(N):
            random_x = np.random.uniform(-max_range/2, max_range/2)
            random_y = np.random.uniform(-max_range/2, max_range/2)
            rv.append((point[0] + random_x, point[1] + random_y))
        return rv

def in_range(point, bounds):
    max_y, max_x = bounds
    y, x = point
    if x < 0 or y < 0:
        return False
    if x >= max_x or y >= max_y:
        return False
    return True

def close_to(gridmap, idx_tuple, radius):
    height, width = gridmap.shape
    y, x = idx_tuple[0], idx_tuple[1]
    y_lo, y_hi = clip(0, height-1, y-radius), clip(0, height-1, y+radius)
    x_lo, x_hi = clip(0, width-1, x-radius), clip(0, width-1, x+radius)

    for i in range(y_lo, y_hi+1):
        for j in range(x_lo, x_hi+1):
            if gridmap[i, j] != 0 and l2_dist((i, j), idx_tuple) < radius:
                #print(i, j)
                return True

    return False

def has_obstacle(gridmap, idx_tuple):
    y, x = idx_tuple[0], idx_tuple[1]
    if gridmap[y, x] != 0:
        return True
    return False

def plan(gridmap, start, goal, radius):
    rv = GlobalPath()
    pq = []
    parents = {}
    dists = {}

    for i in range(len(gridmap)):
        for j in range(len(gridmap[0])):
            dists[(i, j)] = 1e9
    dists[start] = 0
    heappush(pq, (0, start))

    found = False

    while not len(pq) == 0:
        curr = heappop(pq)[1]

        if curr == goal:
            found = True
            break
        
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == j == 0:
                    continue

                new_point = (curr[0] + i, curr[1] + j)
                if not in_range(new_point, gridmap.shape) or has_obstacle(gridmap, new_point, radius):
                    continue

                new_dist = dists[curr] + l2_dist(curr, new_point)
                if new_dist < dists[new_point]:
                    heappush(pq, (new_dist + l2_dist(new_point, goal), new_point))
                    dists[new_point] = new_dist
                    parents[new_point] = curr

    if not found:
        return None

    curr = goal
    while curr != start:
        rv.add_point(curr[::-1])
        curr = parents[curr]

    rv.points = rv.points[::-1]

    return rv

def navigation_function(gridmap, goal, radius):
    pq = []
    parents = {}
    dists = np.zeros(gridmap.shape)

    for i in range(len(gridmap)):
        for j in range(len(gridmap[0])):
            dists[i, j] = len(gridmap) + len(gridmap[0])

    dists[goal] = 0
    heappush(pq, (0, goal))

    while not len(pq) == 0:

        curr = heappop(pq)[1]

        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == j == 0:
                    continue

                new_point = (curr[0] + i, curr[1] + j)

                if not in_range(new_point, gridmap.shape):
                    continue
                
                if has_obstacle(gridmap, new_point):
                    continue

                new_dist = dists[curr] + l2_dist(curr, new_point)
                if new_dist < dists[new_point]:
                    heappush(pq, (new_dist, new_point))
                    dists[new_point] = new_dist
                    parents[new_point] = curr

    return dists, parents

def get_path(parents, start, goal):
    rv = GlobalPath()

    curr = start
    while curr != goal:
        rv.add_point(curr[::-1])
        curr = parents[curr]

    rv.add_point(goal)
    return rv
