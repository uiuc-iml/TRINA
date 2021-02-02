import klampt
import klampt.math.se3
import argparse
import scipy as sp
import scipy.optimize as opt
import numpy as np
import sys
import time
import cvxpy as cp
sys.path.append("../../../Motion")
import TRINAConfig


BASE_X_IND = 0
BASE_Y_IND = 1
BASE_R_IND = 3
CODENAME = 'cholera'
LEFT_LINK_NAME = 'left_EE_link'
RIGHT_LINK_NAME = 'right_EE_link'


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
    set_arms_to_default(robot)

    current_config = robot.getConfig()
    theta = current_config[BASE_R_IND]

    left_desired_twist = np.array([0, 0, 0, -1, 0, 0])
    right_desired_twist = np.array([0, 0, 0, 0, 0, 0])

    # Use cvxpy method to find particular solution
    q_dot_part, avail_jac, _ = get_ls_soln(robot, left_desired_twist, right_desired_twist)

    lam_x = 0.1
    lam_theta = 0.1
    nu = 1.0
    u, s, vh = np.linalg.svd(avail_jac)
    # The closer we are to singular, the more the base can move forward without
    # penalty
    hinge_x = lam_x / s[-1]
    hinge_theta = lam_theta / s[-1]
    print("Hinges: ", hinge_x, hinge_theta)
    null_basis = get_null_basis(u, s, vh)
    t = cp.Variable(null_basis.shape[1])
    total_vec = q_dot_part + null_basis @ t
    base_vec = total_vec[-3:]
    R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    local_base_vel = R.T @ base_vec[:2]
    obj = cp.Minimize(cp.pos(local_base_vel[0] - hinge_x)
        + cp.neg(local_base_vel[0])
        + cp.pos(base_vec[-1] - hinge_theta)
        + cp.neg(base_vec[-1] - hinge_theta)
        + nu * cp.norm_inf(total_vec))
    constraints = [base_vec[1] * np.cos(theta) - base_vec[0] * np.sin(theta) == 0]
    prob = cp.Problem(obj, constraints)
    res = prob.solve()
    t = t.value
    print("T: ", t)
    q_dot_part += null_basis @ t
    print("q_dot total: ", q_dot_part)

    left_dofs = 6
    right_dofs = 6
    q_dot = robot.getVelocity()
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i]
    for i, ind in enumerate(TRINAConfig.get_right_active_Dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i + left_dofs]
    for i, ind in enumerate(get_base_dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i + left_dofs + right_dofs]
    robot.setVelocity(q_dot)

    left_full_robot_jac = np.array(
        robot.link(LEFT_LINK_NAME).getJacobian([0,0,0]))
    right_full_robot_jac = np.array(
        robot.link(RIGHT_LINK_NAME).getJacobian([0,0,0]))
    print("Final Left EE Twist: ", left_full_robot_jac @ np.array(q_dot))
    print("Final Right EE Twist: ", right_full_robot_jac @ np.array(q_dot))

    print("Base null movement")
    q_dot_part, _ = null_base_move(robot, np.array([0, 0, 1, 1, 0, 0]))
    print(q_dot_part)
    q_dot = robot.getVelocity()
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i]
    for i, ind in enumerate(TRINAConfig.get_right_active_Dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i + left_dofs]
    for i, ind in enumerate(get_base_dofs(CODENAME)):
        q_dot[ind] = q_dot_part[i + left_dofs + right_dofs]
    left_full_robot_jac = np.array(
        robot.link(LEFT_LINK_NAME).getJacobian([0,0,0]))
    right_full_robot_jac = np.array(
        robot.link(RIGHT_LINK_NAME).getJacobian([0,0,0]))
    print("Final Left EE Twist: ", left_full_robot_jac @ np.array(q_dot))
    print("Final Right EE Twist: ", right_full_robot_jac @ np.array(q_dot))


def get_ls_soln(robot, left_desired_twist, right_desired_twist):
    current_config = robot.getConfig()
    avail_jac = get_jacobian(robot, LEFT_LINK_NAME, RIGHT_LINK_NAME)
    q_dot_part = cp.Variable(avail_jac.shape[1])
    obj = cp.Minimize(cp.norm2(avail_jac @ q_dot_part
        - np.concatenate((left_desired_twist, right_desired_twist)))**2)
    # q_dot_y cos(theta) - q_dot_x sin(theta) = 0
    theta = current_config[BASE_R_IND]
    constraints = [q_dot_part[-2] * np.cos(theta) - q_dot_part[-3] * np.sin(theta) == 0]
    prob = cp.Problem(obj, constraints)
    res = prob.solve()
    return q_dot_part.value, avail_jac, res


def null_base_move(robot, target_base_twist):
    current_config = robot.getConfig()
    theta = current_config[BASE_R_IND]
    jac = get_jacobian(robot, LEFT_LINK_NAME, RIGHT_LINK_NAME)
    u, s, vh = np.linalg.svd(jac)
    null_basis = get_null_basis(u, s, vh)
    t = cp.Variable(null_basis.shape[1])
    total_vec = null_basis @ t
    base_vec = total_vec[-3:]
    base_twist = np.array([0, 0, base_vec[2], base_vec[0], base_vec[1], 0], dtype=object)
    obj = cp.Minimize((target_base_twist[2] - base_vec[2])**2 + (target_base_twist[3] - base_vec[0])**2 + (target_base_twist[4] - base_vec[1])**2)
    constraints = [base_vec[1] * np.cos(theta) - base_vec[0] * np.sin(theta) == 0]
    prob = cp.Problem(obj, constraints)
    res = prob.solve()
    return null_basis @ t.value, res


def get_null_basis(u, s, vh):
    v = vh.T
    v_extra_cols = max(v.shape[1] - s.shape[0], 0)
    nullity = v_extra_cols
    null_inds = []
    for i, val in enumerate(s):
        if val == 0:
            nullity += 1
            null_inds.append(i)
    null_basis = np.empty((v.shape[0], nullity))
    null_basis[:, :v_extra_cols] = v[:, -v_extra_cols:]
    for i, ind in null_inds:
        null_basis[:, v_extra_cols + i] = v[:, ind]
    return null_basis

def get_jacobian(robot, left_link_name, right_link_name):
    left_full_robot_jac = np.array(
        robot.link(left_link_name).getJacobian([0,0,0]))
    right_full_robot_jac = np.array(
        robot.link(right_link_name).getJacobian([0,0,0]))
    left_dofs = 6
    right_dofs = 6
    base_dofs = 3
    total_dofs = left_dofs + right_dofs + base_dofs
    avail_jac = np.zeros((6*2, total_dofs)) # Each column has 6 elts
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
        avail_jac[:6, i] = np.array(left_full_robot_jac[:, ind])
    for i, ind in enumerate(TRINAConfig.get_right_active_Dofs(CODENAME)):
        avail_jac[6:, i+left_dofs] = np.array(right_full_robot_jac[:, ind])
    for i, ind in enumerate(get_base_dofs(CODENAME)):
        avail_jac[:6, i+left_dofs+right_dofs] = np.array(left_full_robot_jac[:, ind])
        avail_jac[6:, i+left_dofs+right_dofs] = np.array(right_full_robot_jac[:, ind])
    return avail_jac


def set_arms_to_default(robot):
    cfg = robot.getConfig()
    for i, ind in enumerate(TRINAConfig.get_left_active_Dofs(CODENAME)):
       cfg[ind] = TRINAConfig.left_untucked_config[i]
    for i, ind in enumerate(TRINAConfig.get_right_active_Dofs(CODENAME)):
       cfg[ind] = TRINAConfig.right_untucked_config[i]
    robot.setConfig(cfg)


def get_base_dofs(codename):
    # Hard coded for cholera, will eventually support the other versions
    return [0, 1, 3]


if __name__ == "__main__":
    main()
