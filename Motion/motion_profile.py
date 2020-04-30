import math

class State:

    def __init__(self, x, y, theta, v, w, acc, t):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w
        self.acc = acc
        self.t = t

    def __repr__(self):
        return "({}, {})".format(self.x, self.y)

"""
TODO: this currently doesn't really take in curvature to make the robot
slow down around curves
"""
def generate_profile(local_path, acc, max_v, dt, end_v = 0, start_v = 0):
    total_dist = local_path.get_length()

    max_v = min(max_v, math.sqrt(start_v**2 + end_v**2)/2.0 + total_dist*acc)

    accel_t = (max_v - start_v)/acc
    decel_t = (max_v - end_v)/acc
    acc_dist = start_v*accel_t + (0.5*acc*accel_t**2)
    decel_dist = max_v*decel_t - (0.5*acc*decel_t**2)

    cruise_dist = total_dist - acc_dist - decel_dist
    cruise_t = cruise_dist/max_v
    total_t = cruise_t + accel_t + decel_t

    num_acc_points = int(accel_t/dt)
    num_cruise_points = int(cruise_t/dt)
    num_decel_points = int(decel_t/dt)
    total_points = int(total_t/dt)

    curr_time = 0
    prev_theta = 0
    rv = []

    for i in range(num_acc_points):
        s = float(i)/float(total_points)
        x, y, theta = local_path.get_pose(s)
        v = start_v + (acc*curr_time)
        w = local_path.get_curvature(s) * v
        s = State(x, y, theta, v, w, acc, curr_time)
        rv.append(s)
        curr_time += dt
        prev_theta = theta

    for i in range(num_cruise_points):
        s = float(i + num_acc_points)/float(total_points)
        x, y, theta = local_path.get_pose(s)
        v = max_v
        w = local_path.get_curvature(s) * v
        s = State(x, y, theta, v, w, 0, curr_time)
        rv.append(s)
        curr_time += dt
        prev_theta = theta

    for i in range(num_decel_points):
        s = float(i + num_cruise_points + num_acc_points)/float(total_points)
        x, y, theta = local_path.get_pose(s)
        adj_t = curr_time - (accel_t + cruise_t)
        v = max_v - acc*adj_t
        w = local_path.get_curvature(s) * v
        s = State(x, y, theta, v, w, -acc, curr_time)
        rv.append(s)
        curr_time += dt
        prev_theta = theta

    return rv
