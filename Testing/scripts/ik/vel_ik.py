import klampt
import klampt.math.se3
import argparse
import scipy as sp
import scipy.optimize as opt
import numpy as np
import sys
import time
sys.path.append("../../../Motion")
import TRINAConfig


BASE_R_IND = 3
CODENAME = 'cholera'
LINK_NAME = 'left_EE_link'


def main():
    parser = argparse.ArgumentParser(description="Test more general "
        + "ik penalties")
    parser.add_argument("-p", type=str, default="../../../Motion/data/",
        help="path to world file")
    parser.add_argument("-w", type=str, default="TRINA_world_cholera.xml",
        help="world file name")
    args = parser.parse_args()

    world = klampt.WorldModel()
    fn = args.p + args.w
    res = world.loadFile(fn)
    if not res:
        logger.error(f"Couldn't load file {fn}, exiting.")
        sys.exit()
    robot = world.robot(0)
    set_left_arm_to_default(robot)

    # Start with no rotation, just move in the x direction
    desired_vel = np.array([0, 0, 0, 0, 1, 0])

    # Use std lstsq method to find particular solution
    full_robot_jac = np.array(robot.link(LINK_NAME).getJacobian([0,0,0]))
    avail_jac = np.empty((6, 7)) # Each column has 6 elts, we use 7 DoFs
    avail_jac[:, 0] = full_robot_jac[:, BASE_R_IND]
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
        avail_jac[:, i+1] = np.array(full_robot_jac[:, ind])
    q_dot_part = np.linalg.lstsq(avail_jac, desired_vel, rcond=None)[0]
    print(q_dot_part)

    q_dot = robot.getVelocity()
    q_dot[BASE_R_IND] = q_dot_part[0]
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i+1]
    robot.setVelocity(q_dot)
    print(np.array(robot.link(LINK_NAME).getJacobian([0,0,0])) @ np.array(q_dot))


def set_left_arm_to_default(robot):
    cfg = robot.getConfig()
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
       cfg[ind] = TRINAConfig.left_untucked_config[i]
    robot.setConfig(cfg)


if __name__ == "__main__":
    main()
