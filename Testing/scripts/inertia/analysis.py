import scipy.optimize as opt
import numpy as np
import pickle
import argparse
import klampt
import matplotlib.pyplot as plt
import sys
sys.path.append("../../../Motion")
import TRINAConfig
import logging
logging.basicConfig()


def main():
    parser = argparse.ArgumentParser(description="Calibrate the inertia "
        + "matrix, center of mass, and mass of the gripper")
    parser.add_argument("inp", type=str, help="input file")
    parser.add_argument("-p", type=str, default="../../../Motion/data/",
        help="path to world file")
    parser.add_argument("-w", type=str, default="TRINA_world_cholera.xml",
        help="world file name")
    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.INFO)

    # Super jank way to get the name assuming a given world file format
    codename = args.w.split("_")[-1].split(".")[0]
    logger.info("Codename: " + str(codename))

    p = 'position'
    v = 'velocity'
    w = 'wrench'
    t = 'time'
    with open(args.inp, "rb") as inp:
        # The input file contains a list of dictionaries recording time stamps,
        # joint positions and velocities, and the wrench at the end effector:
        # keys:
        #   'time'
        #   'position'
        #   'velocity'
        #   'wrench' -- wrench is force, then torque
        data = pickle.load(inp)
    world = klampt.WorldModel()
    fn = args.p + args.w
    res = world.loadFile(fn)
    if not res:
        logger.error(f"Couldn't load file {fn}, exiting.")
        sys.exit()
    robot = world.robot(0)
    logger.info("Loaded world and robot")
    dofs = TRINAConfig.get_left_active_Dofs(codename)
    ee_link_name = "left_EE_link"
    times = []
    R_vals = []
    p_vals = []
    omega_vals = []
    v_vals = []
    d_omega_vals = []
    d_v_vals = []
    w_vals = [] # torque, force
    for i, meas in enumerate(data):
        config = robot.getConfig()
        for j, ind in enumerate(dofs):
            config[ind] = meas[p][j]
        robot.setConfig(config)
        link = robot.link(ee_link_name)
        jac = np.array(link.getJacobian([0,0,0]))
        if i == 0:
            # Initialize some boundary values
            last_t = meas[t]
            last_vels = robot.getVelocity()
            for j, ind in enumerate(dofs):
                last_vels[ind] = meas[v][j]
            last_ee_twist = jac @ np.array(last_vels)
        else:
            # Ignore the first data point since we don't have any history
            # information to get acceleration data
            vels = robot.getVelocity()
            for j, ind in enumerate(dofs):
                vels[ind] = meas[v][j]
            ee_transform = link.getTransform()
            ee_twist = jac @ np.array(vels)
            dt = meas[t] - last_t
            # Compute last twist in this frame to compute acceleration
            # (everything in same frame)
            last_ee_twist = jac @ np.array(last_vels)
            # Compute accel
            ee_accel = (ee_twist - last_ee_twist) / dt
            ee_wrench = np.array(meas[w])

            times.append(meas[t])
            R_vals.append(ee_transform[0])
            p_vals.append(ee_transform[1])
            omega_vals.append(ee_twist[:3])
            v_vals.append(ee_twist[3:])
            d_omega_vals.append(ee_accel[:3])
            d_v_vals.append(ee_accel[3:])
            w_vals.append(ee_wrench)
            # w_vals.append(np.zeros(6))

            last_t = meas[t]
            last_vels = vels[:]
            last_ee_twist = ee_twist[:]
    logger.info("Finished loading data, starting optimization")
    start = np.array([0.0084, 0, 0, 0.0064, 0, 0.0023, 0, 0, 0.0624, 1.0182])
    # start = np.array([ 4.96670394e-02,  9.18877726e-02,  1.73765360e-01,  9.33424707e-01,
    #     1.47458136e-01,  1.30701951e-16, 0, 0,
    #    0.8,  9.48624154e-04])
    # res = opt.minimize(
    #     objective(times, R_vals, p_vals, omega_vals, d_omega_vals, v_vals,
    #         d_v_vals, w_vals),
    #     start,
    #     constraints=[
    #         {'type':'ineq', 'fun': lambda x: x[-1]},    # Mass is nonnegative
    #         {'type':'ineq', 'fun': lambda x: x[0]},     # So are diags of I matrix
    #         {'type':'ineq', 'fun': lambda x: x[3]},
    #         {'type':'ineq', 'fun': lambda x: x[5]},
    #         {'type':'eq', 'fun': lambda x: x[1]},       # Try diag I matrix
    #         {'type':'eq', 'fun': lambda x: x[2]},
    #         {'type':'eq', 'fun': lambda x: x[4]},
    #     ])
    # tmp = objective(times, R_vals, p_vals, omega_vals, d_omega_vals, v_vals,
    #         d_v_vals, w_vals)
    # x_vals = np.linspace(-2, 2, 100)
    # y_vals = np.zeros(len(x_vals))
    # for i, x in enumerate(x_vals):
    #     v = start
    #     v[-1] = x
    #     y_vals[i] = tmp(v)
    y_vals = np.linalg.norm(ee_error(start, times, R_vals, p_vals,
        omega_vals, d_omega_vals, v_vals, d_v_vals, w_vals)[0], axis=1)
    y_vals, preds, targs = ee_error(start, times, R_vals, p_vals,
            omega_vals, d_omega_vals, v_vals, d_v_vals, w_vals)
    # plt.plot(y_vals)
    plt.plot(preds, label="Pred")
    # plt.plot(targs, label="Targets")
    # plt.plot(omega_vals, label="Omega")
    # plt.plot(d_omega_vals, label="D_Omega")
    # plt.plot(v_vals, label="V")
    # plt.plot(d_v_vals, label="D_V")
    # plt.plot(w_vals, label="Wrench")
    plt.legend(loc='lower left')
    plt.show()
    logger.info("Finished optimization:")
    logger.info(str(res))


def objective(t_s, R_s, p_s, omega_s, d_omega_s, v_s, d_v_s, w_s):
    """inp[0:6] Elements of inertia matrix
    0 1 2
      3 4
        5
    inp[6:9] EE --> COM
    inp[9] m
    """
    def f(inp):
        errors = ee_error(inp, t_s, R_s, p_s, omega_s, d_omega_s, v_s, d_v_s, w_s)
        # Important: Averaging over number of measurements was necessary for
        # achieving convergence, perhaps the slopes change too quickly without
        # it.
        return np.mean(np.linalg.norm(errors, axis=1)**2)
    return f


def error(inp, t_s, R_s, p_s, omega_s, d_omega_s, v_s, d_v_s, w_s):
    I = np.array([
        [inp[0], inp[1], inp[2]],
        [inp[1], inp[3], inp[4]],
        [inp[2], inp[4], inp[5]]
    ])
    r = inp[6:9]
    m = inp[9]
    errors = np.empty((len(t_s), 6))
    targets = np.empty((len(t_s), 6))
    predictions = np.empty((len(t_s), 6))
    g = np.array([0, 0, -9.81])
    grav = np.array([m*g, np.zeros(3)]).flatten()
    init_p = np.array(p_s[0])
    init_R = np.array(R_s[0]).reshape(-1,3).T
    tare_offset = np.concatenate((m*g,
        np.cross(init_p + (init_R @ r), m*g)))
    print("Tare offset: ", tare_offset)
    # tare_offset = np.zeros(6)
    for i, t in enumerate(t_s):
        R = np.array(R_s[i]).reshape(-1, 3).T
        p = np.array(p_s[i])
        Ic = R @ I @ R.T
        accel_mat = np.zeros((6, 6))
        accel_mat[:3, :3] = m * np.eye(3)
        accel_mat[3:, 3:] = Ic

        d_omega = np.array(d_omega_s[i])
        d_v = np.array(d_v_s[i])
        accel = np.array([d_v + np.cross(d_omega, p + R @ r),
            d_omega]).flatten()

        omega = np.array(omega_s[i])
        c_vec = np.cross(omega, Ic @ omega)
        c_vec = np.array([np.zeros(3), c_vec]).flatten()

        f = np.array(w_s[i][:3]) + tare_offset[:3]
        tau = np.array(w_s[i][3:]) + tare_offset[3:]
        target = np.array([f, tau
            - np.cross(p + R @ r, m * g)]).flatten()
        pred = accel_mat @ accel + c_vec + grav
        errors[i] = pred - target
        predictions[i] = pred
        targets[i] = target
    return errors, predictions, targets


def ee_error(inp, t_s, R_s, p_s, omega_s, d_omega_s, v_s, d_v_s, w_s):
    I = np.array([
        [inp[0], inp[1], inp[2]],
        [inp[1], inp[3], inp[4]],
        [inp[2], inp[4], inp[5]]
    ])
    r = inp[6:9]
    m = inp[9]
    errors = np.empty((len(t_s), 6))
    targets = np.empty((len(t_s), 6))
    predictions = np.empty((len(t_s), 6))
    g = np.array([0, 0, -9.81])
    init_p = np.array(p_s[0])
    init_R = np.array(R_s[0]).reshape(-1,3).T
    tare_offset = np.concatenate((m * init_R.T @ g,
        np.cross(r, m * init_R.T @ g)))
    print("Tare offset: ", tare_offset)
    # tare_offset = np.zeros(6)
    for i, t in enumerate(t_s):
        R = np.array(R_s[i]).reshape(-1, 3).T
        p = np.array(p_s[i])
        accel_mat = np.zeros((6, 6))
        accel_mat[:3, :3] = m * np.eye(3)
        accel_mat[3:, 3:] = I

        d_omega = np.array(d_omega_s[i])
        d_v = np.array(d_v_s[i])
        accel = np.array([R.T @ d_v,
            R.T @ d_omega]).flatten()

        omega = np.array(omega_s[i])
        c_vec = np.cross(R.T @ omega, I @ R.T @ omega)
        c_vec = np.array([np.zeros(3), c_vec]).flatten()

        f = np.array(w_s[i][:3]) + tare_offset[:3]
        tau = np.array(w_s[i][3:]) + tare_offset[3:]
        target = np.array([f, tau]).flatten()
        grav = np.concatenate((m * R.T @ g, np.cross(r, m * R.T @ g)))
        pred = accel_mat @ accel + c_vec + grav
        errors[i] = pred - target
        predictions[i] = pred
        targets[i] = target
    return errors, predictions, targets


if __name__ == "__main__":
    main()
