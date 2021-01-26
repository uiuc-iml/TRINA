import klampt
import klampt.math.se3
import argparse
import scipy as sp
import scipy.optimize as opt
import numpy as np
import sys
sys.path.append("../../../Motion")
import TRINAConfig


BASE_R_IND = 3
CODENAME = 'cholera'


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
    start = robot.getConfig()
    start = np.random.rand(len(start))
    print("Start base rotation: ", start[BASE_R_IND])
    init_guess = np.zeros(7)
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
        init_guess[i+1] = start[ind]
    init_guess[0] = start[BASE_R_IND]
    robot.setConfig(start)
    delta = list(klampt.math.se3.identity())
    delta[1] = [-0.001, 0, 0]
    link_name = 'left_EE_link'
    current_pos = robot.link(link_name).getTransform()
    target = klampt.math.se3.mul(delta, current_pos)
    lam = 0.1
    res = opt.minimize(
        ik_obj(robot.getConfig(), link_name, target, robot, lam),
        np.zeros(7), method='Newton-CG',
        jac=ik_jac(robot.getConfig(), link_name, target, robot, lam))
    print(res)


def ik_obj(q, link_name, target, robot, lam):
    def f(x):
        """x is a 7 element vector representing the
            [0] -> base rotation about the world z axis
            [1:] -> joint angles of the arms
        """
        obj = lam * abs(x[0] - q[BASE_R_IND])
        return obj
    return f


def ik_jac(q, link_name, target, robot, lam):
    def jac(x):
        """x is a 7 element vector representing the
            [0] -> base rotation about the world z axis
            [1:] -> joint angles of the arms
        """
        relevant_grad = np.zeros(7)
        relevant_grad[0] += lam * np.sign(x[0] - q[BASE_R_IND])
        return relevant_grad
    return jac


if __name__ == "__main__":
    main()
