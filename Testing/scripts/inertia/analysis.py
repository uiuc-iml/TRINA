import scipy.optimize as opt
import numpy as np
import pickle
import argparse
import klampt
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
            # Compute accel for the learner
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
            #w_vals.append(np.zeros(6))

            last_t = meas[t]
            last_ee_twist = ee_twist
    logger.info("Finished loading data, starting optimization")
    res = opt.minimize(
        objective(times, R_vals, p_vals, omega_vals, d_omega_vals, v_vals,
            d_v_vals, w_vals),
        np.zeros(10))
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
        I = np.array([
            [inp[0], inp[1], inp[2]],
            [inp[1], inp[3], inp[4]],
            [inp[2], inp[4], inp[5]]
        ])
        r = inp[6:9]
        m = inp[9]
        obj = 0
        g = np.array([0, 0, -9.81])
        grav = np.array([m*g, np.zeros(3)]).flatten()
        init_p = np.array(p_s[0])
        init_R = np.array(R_s[0]).reshape(-1,3).T
        tare_offset = np.concatenate((m*g,
            np.cross((init_p + init_R @ r), m*g)))
        for i, t in enumerate(t_s):
            R = np.array(R_s[i]).reshape(-1, 3).T
            p = np.array(p_s[i])
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

            f = w_s[i][0:3]
            tau = w_s[i][3:6]
            target = np.array([f, tau
                - (p + R @ (m * np.cross(r, g)))]).flatten() - tare_offset
            obj += np.linalg.norm(accel_mat @ accel + c_vec + grav - target)**2
        # Important: Averaging over number of measurements was necessary for
        # achieving convergence, perhaps the slopes change too quickly without
        # it.
        return obj / len(t_s)
    return f


if __name__ == "__main__":
    main()
