import pickle
import argparse
import numpy as np
import logging
import klampt
import sys
sys.path.append("../../../Motion")
import TRINAConfig


# 9 elements of the rotation matrix, 3 for the translation of the EE,
# 6 + 6 for the velocity from the last two measurements, 6 for the computed
# acceleration, 1 for the time delay between the measurements
num_inp_features = 9 + 3 + 6 + 6 + 6 + 1
num_out_features = 6


def main():
    parser = argparse.ArgumentParser(description="Learn model to predict "
        + "external force")
    parser.add_argument("inp", type=str, help="input file")
    parser.add_argument("-p", type=str, default="../../../Motion/data/",
        help="path to world file")
    parser.add_argument("-w", type=str, default="TRINA_world_cholera.xml",
        help="world file name")
    parser.add_argument("-t", type=int, default=100,
        help="number of measurements to average to compute tear, use first "
        + "`tear_num` readings")
    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.WARNING)

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
    tear_value = np.zeros(6)
    if args.t <= len(data):
        for i in range(args.t):
            tear_value += np.array(data[i][w])
        tear_value /= args.t
    else:
        logger.warn(f"Skipped tear because tear_num={args.t} was greater "
            + f"than the number of datapoints ({len(data)})")
    world = klampt.WorldModel()
    fn = args.p + args.w
    res = world.loadFile(fn)
    if not res:
        logger.error(f"Couldn't load file {fn}, exiting.")
        sys.exit()
    robot = world.robot(0)
    dofs = TRINAConfig.get_left_active_Dofs(codename)
    ee_link_name = "left_EE_link"
    data_array = np.empty((len(data) - 1, num_inp_features + num_out_features))
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
            ee_accel = (ee_twist - last_ee_twist) / dt
            ee_wrench = np.array(meas[w]) - tear_value
            data_array[i - 1] = np.concatenate([np.array(ee_transform[0]),
                np.array(ee_transform[1]), ee_twist,
                last_ee_twist, ee_accel, np.array(dt).reshape(-1),
                ee_wrench])

            last_t = meas[t]
            last_ee_twist = ee_twist
    np.save('data.npy', data_array)
    np.save('tear.npy', tear_value)


if __name__ == "__main__":
    main()
