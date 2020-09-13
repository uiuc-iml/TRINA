import math

class Spline2d():
    # pose = [x, y, theta (rad)]
    def __init__(self, start_pose, end_pose):
        x0 = start_pose[0]
        x1 = end_pose[0]
        y0 = start_pose[1] 
        y1 = end_pose[1] 
        dx = x1 - x0
        dy = y1 - y0
        self.dist = math.sqrt(dx**2 + dy**2)
        scale = 1.5 * self.dist
        vx0 = scale*math.cos(start_pose[2])
        vx1 = scale*math.cos(end_pose[2])
        vy0 = scale*math.sin(start_pose[2])
        vy1 = scale*math.sin(end_pose[2])

        ax = -2.0*x1 + 2.0*x0 + vx1 + vx0
        ay = -2.0*y1 + 2.0*y0 + vy1 + vy0
        bx = 3.0*x1 - 3.0*x0 - vx1 - 2.0*vx0
        by = 3.0*y1 - 3.0*y0 - vy1 - 2.0*vy0
        cx = vx0
        cy = vy0
        dx = x0
        dy = y0

        self.x_coeffs = [ax, bx, cx, dx]
        self.y_coeffs = [ay, by, cy, dy]
        self.length = self.get_length()

    def get_pose(self, s):
        x = self.x_coeffs[0]*s**3 + self.x_coeffs[1]*s**2 + self.x_coeffs[2]*s + self.x_coeffs[3]
        y = self.y_coeffs[0]*s**3 + self.y_coeffs[1]*s**2 + self.y_coeffs[2]*s + self.y_coeffs[3]
        dx = 3*self.x_coeffs[0]*s**2 + 2*self.x_coeffs[1]*s + self.x_coeffs[2]
        dy = 3*self.y_coeffs[0]*s**2 + 2*self.y_coeffs[1]*s + self.y_coeffs[2]
        theta = math.atan2(dy, dx)
        return (x, y, theta)

    def get_curvature(self, s):
        dx = 3*self.x_coeffs[0]*s**2 + 2*self.x_coeffs[1]*s + self.x_coeffs[2]
        dy = 3*self.y_coeffs[0]*s**2 + 2*self.y_coeffs[1]*s + self.y_coeffs[2]
        ddx = 6*self.x_coeffs[0]*s + 2*self.x_coeffs[1]
        ddy = 6*self.y_coeffs[0]*s + 2*self.y_coeffs[1]
        curvature = (ddy*dx - ddx*dy)/(math.pow(dx*dx+dy*dy, 1.5))
        return curvature

    def get_length(self):
        dist = 0
        num_steps = 1000
        for i in range(num_steps - 1):
            i = float(i)
            curr_x, curr_y, _ = self.get_pose(i/num_steps)
            next_x, next_y, _ = self.get_pose((i+1)/num_steps)
            dx = next_x - curr_x
            dy = next_y - curr_y
            dist += math.sqrt(dx**2 + dy**2)
        return dist

    def get_pose_by_dist(self, dist):
        return self.get_pose(dist/self.length)

class LocalPath():

    # points - list of poses
    def __init__(self, points):
        self.splines = []
        self.length = 0
        self.points = points
        for i in range(len(points)-1):
            spline = Spline2d(points[i], points[i+1])
            self.splines.append(spline)
            self.length += spline.get_length()

    def get_curvature(self, s):
        dist = self.length*s
        cum_dist = 0
        for i in range(len(self.splines)):
            spline = self.splines[i]
            if dist < cum_dist + spline.length:
                return spline.get_curvature((dist - cum_dist)/spline.length)
            cum_dist += spline.length
        return self.splines[-1].get_curvature(1.0)

    def get_pose(self, s):
        dist = self.length*s
        cum_dist = 0
        for i in range(len(self.splines)):
            spline = self.splines[i]
            if dist < cum_dist + spline.length:
                return spline.get_pose((dist - cum_dist)/spline.length)
            cum_dist += spline.length
        return self.splines[-1].get_pose(1.0)

    def get_pose_by_dist(self, dist):
        return self.get_pose(dist/self.length)

    def get_xytheta(self, num_points):
        xs, ys, thetas = [], [], []
        for i in range(num_points):
            pose = self.get_pose(float(i)/num_points)
            xs.append(pose[0])
            ys.append(pose[1])
            thetas.append(pose[2])
        return xs, ys, thetas

    def collides(self, gridmap):
        xs, ys, _ = self.get_xytheta(200)
        for i in range(len(xs)):
            x = int(xs[i])
            y = int(ys[i])
            if gridmap[y, x] != 0:
                return True
        return False

    def get_length(self):
        rv = 0
        for s in self.splines:
            rv += s.get_length()
        return rv

    def __eq__(self, other):
        if other is None:
            return False
        rv = set(self.points) == set(other.points)
        return rv
