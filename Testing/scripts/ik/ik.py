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
    delta[1] = [-0.01, 0, 0]
    link_name = 'left_EE_link'
    current_pos = robot.link(link_name).getTransform()
    target = klampt.math.se3.mul(delta, current_pos)
    lam = 0.1
    res = opt.minimize(
        ik_obj(robot.getConfig(), link_name, target, robot, lam),
        init_guess, method='Newton-CG',
        jac=ik_jac(robot.getConfig(), link_name, target, robot, lam))
    print(res)


def ik_obj(q, link_name, target, robot, lam):
    def f(x):
        """x is a 7 element vector representing the
            [0] -> base rotation about the world z axis
            [1:] -> joint angles of the arms
        """
        # obj = lam * abs(x[0] - q[BASE_R_IND])
        obj = lam * (x[0] - q[BASE_R_IND])**2
        config = q[:]
        config[BASE_R_IND] = x[0]
        for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
            config[ind] = x[i+1]
        robot.setConfig(config)
        ee_t = robot.link(link_name).getTransform()
        error = np.array(klampt.math.se3.error(ee_t, target))
        obj += np.linalg.norm(error)
        print(obj, lam * abs(x[0] - q[BASE_R_IND]))
        return obj
    return f


def ik_jac(q, link_name, target, robot, lam):
    def jac(x):
        """x is a 7 element vector representing the
            [0] -> base rotation about the world z axis
            [1:] -> joint angles of the arms
        """
        config = q[:]
        config[BASE_R_IND] = x[0]
        for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
            config[ind] = x[i+1]
        robot.setConfig(config)
        ee_t = robot.link(link_name).getTransform()
        error = np.array(klampt.math.se3.error(ee_t, target))
        grad = np.linalg.lstsq(
            robot.link(link_name).getJacobian([0,0,0]), error, rcond=None)
        grad = grad[0]
        relevant_grad = np.zeros(len(x))
        relevant_grad[0] = grad[BASE_R_IND]
        for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
            relevant_grad[i+1] = grad[ind]
        # relevant_grad[0] += lam * np.sign(x[0] - q[BASE_R_IND])
        relevant_grad[0] += lam * 2 * (x[0] - q[BASE_R_IND])
        print(lam * np.sign(x[0] - q[BASE_R_IND]), x[0], q[BASE_R_IND])
        return relevant_grad
    return jac


if __name__ == "__main__":
    main()
