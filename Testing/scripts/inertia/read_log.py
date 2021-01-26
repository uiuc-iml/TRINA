import pandas as pd
import json
import numpy as np
import functools

from scipy.optimize import minimize


def main():
    frame = pd.read_csv('../../../logs/2020_12_20_20_01_50', sep='|')
    robot_data = frame[frame['nature'] == 'robot']
    print("Robot data points", len(robot_data))
    side = 'LeftArm' # or RightArm
    w = 'EEWrench'
    f = 'global'
    v = 'VelocityEE'
    p = 'PositionEE'
    times = []
    R_vals = []
    omega_vals = []
    v_vals = []
    d_omega_vals = []
    d_v_vals = []
    w_vals = [] # torque, force
    for idx, row in robot_data.iterrows():
        data = json.loads(row.log.replace("'", '"'))
        complete = True
        for key in [w, v, p]:
            if key not in data:
                complete = False
                break
        if complete:
            times.append(row.trina_time)
            R_vals.append(data[p][side][0])
            # v, omega
            twist = data[v][side]
            v_vals.append(twist[0])
            omega_vals.append(twist[1])
            w_vals.append(data[w][side][f])
    print("Number of complete points: ", len(times))
    print(len(omega_vals), len(v_vals))
    for i in range(1, len(times)-1):
        fd = i + 1
        bk = i - 1
        denom = (times[fd] - times[bk])
        d_omega = (np.array(omega_vals[fd])
            - np.array(omega_vals[bk])) /denom
        d_v = (np.array(v_vals[fd])
            - np.array(v_vals[bk])) / denom
        d_omega_vals.append(d_omega)
        d_v_vals.append(d_v)
    times.pop(0); times.pop()
    R_vals.pop(0); R_vals.pop()
    omega_vals.pop(0); omega_vals.pop()
    v_vals.pop(0); v_vals.pop()
    w_vals.pop(0); w_vals.pop()
    s_obj = functools.partial(objective,
        t_s=times, R_s=R_vals, omega_s=omega_vals,
        d_omega_s=d_omega_vals, v_s=v_vals, d_v_s=d_v_vals,
        w_s=w_vals)
    print(s_obj(np.zeros(10)))
    print(minimize(s_obj, np.zeros(10)))


def objective(inp, t_s, R_s, omega_s, d_omega_s, v_s, d_v_s, w_s):
    """inp[0:6] Elements of inertia matrix
    0 1 2
      3 4
        5
    inp[6:9] EE --> COM
    inp[9] m
    """
    I = np.array([
        [inp[0], inp[1], inp[2]],
        [inp[1], inp[3], inp[4]],
        [inp[2], inp[4], inp[5]]
    ])
    r = inp[6:9]
    m = inp[9]
    obj = 0
    for i, t in enumerate(t_s):
        R = np.array(R_s[i]).reshape(-1, 3).T
        Ic = R.T @ I @ R
        accel_mat = np.zeros((6, 6))
        accel_mat[0:3, 0:3] = m * np.eye(3)
        accel_mat[3:6, 3:6] = Ic

        d_omega = np.array(d_omega_s[i])
        d_v = np.array(d_v_s[i])
        accel = np.array([d_v + np.cross(d_omega, R @ r),
            d_omega]).flatten()

        omega = np.array(omega_s[i])
        c_vec = np.cross(omega, Ic @ omega)
        c_vec = np.array([np.zeros(3), c_vec]).flatten()

        g = np.array([0, 0, -9.81])
        grav = np.array([m*g, np.zeros(3)]).flatten()

        f = w_s[i][0:3]
        tau = w_s[i][3:6]
        target = np.array([f, tau - R @ (m * np.cross(r, g))]).flatten()
        obj += np.linalg.norm(accel_mat @ accel + c_vec + grav - target)**2
    return obj


if __name__ == "__main__":
    main()
