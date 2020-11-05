import numpy as np
import math

from .utils import *

class Circle:

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def in_range(self, point):
        dist = l2_dist(point, self.center)
        return dist < self.radius

    def collides(self, gridmap):
        height, width = gridmap.shape

        lo_x = int(self.center[0] - self.radius)
        hi_x = int(self.center[0] + self.radius)
        lo_y = int(self.center[1] - self.radius)
        hi_y = int(self.center[1] + self.radius)

        for x in range(lo_x, hi_x):
            for y in range(lo_y, hi_y):
                if y >= height or y < 0:
                    continue
                if x >= width or x < 0:
                    continue

                if gridmap[y, x] != 0:
                    if self.in_range((x, y)):
                        return True
        return False

    def plot(self, plt):
        circle = plt.Circle(self.center, self.radius, color='blue')
        plt.gcf().gca().add_artist(circle)
