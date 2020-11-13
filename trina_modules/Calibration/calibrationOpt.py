import klampt
from klampt.math import vectorops,so3,se3
import numpy as np
from klampt import WorldModel,vis
from utils import *
import math
def URDFCalibration(Tl,ql,Tr,qr,T_marker_1,T_c_l_0,T_c_r_0,world_path,URDF_save_path,links):
    """
    Optimize for the transforms of thea arms and the wrist-mounted cameras

    Since the initial guess would be close, use local solve only for now
    """
   
    #first load the initial guess for the arm transforms
    world = WorldModel()
    res = world.readFile(world_path)
    if not res:
        raise RuntimeError("unable to load model")

    robot_model = world.robot(0)
    link_base_l = robot_model.link(6)
    link_base_r = robot_model.link(14)
    T_l_0 = link_base_l.getTransform()
    T_r_0 = link_base_r.getTransform()

    x_l_0 = T_l_0[1] + so3.moment(T_l_0[0]) + T_c_l_0[1] + so3.moment(T_c_l_0[0])
    x_r_0 = T_r_0[1] + so3.moment(T_r_0[0]) + T_c_r_0[1] + so3.moment(T_c_r_0[0])

    def funcl(x):
        """
        x: [0:6] pos + moment of shoulder [6:12] pos + moment of camera
        """
        expr=0.0
        x = x.tolist()
        T_base = (so3.from_moment(x[3:6]),x[0:3])
        T_c_EE = (so3.from_moment(x[9:12]),x[6:9])
        for T_m_c,q in zip(Tl,ql):
            T_EE = getLeftLinkTransform(robot_model,q,links[0])
            T_EE_base = se3.mul(se3.inv(T_l_0),T_EE) #EE in arm base
            T_m_predicted = se3.mul(se3.mul(se3.mul(T_base,T_EE_base),T_c_EE),T_m_c)
            error = vectorops.normSquared(se3.error(T_marker_1,T_m_predicted))
            expr += error
        expr = math.sqrt(expr/(len(Tl)))
        #print("Local solve: func(left arm)=%f"%expr)
        return expr

    def funcr(x):
        """
        x: [0:6] pos + moment of shoulder [6:12] pos + moment of camera
        """
        expr=0.0
        x = x.tolist()
        T_base = (so3.from_moment(x[3:6]),x[0:3])
        T_c_EE = (so3.from_moment(x[9:12]),x[6:9])
        for T_m_c,q in zip(Tr,qr):
            T_EE = getRightLinkTransform(robot_model,q,links[1])
            T_EE_base = se3.mul(se3.inv(T_r_0),T_EE) #EE in arm base
            T_m_predicted = se3.mul(se3.mul(se3.mul(T_base,T_EE_base),T_c_EE),T_m_c)
            error = vectorops.normSquared(se3.error(T_marker_1,T_m_predicted))
            expr += error
        expr = math.sqrt(expr/(len(Tr)))
        return expr


    import scipy.optimize as sopt
    res = sopt.minimize(funcl,np.array(x_l_0),method='L-BFGS-B')
    x_l = res.x
    print("Local solve: func(left arm)=%f"%funcl(x_l))
    res = sopt.minimize(funcr,np.array(x_r_0),method='L-BFGS-B')
    x_r = res.x
    print("Local solve: func(right arm)=%f"%funcr(x_r))

    #left shoulder, left camera, right shoulder, right camera
    return (so3.from_moment(x_l[3:6]),x_l[0:3]),(so3.from_moment(x_l[9:12]),x_l[6:9]),\
        (so3.from_moment(x_r[3:6]),x_r[0:3]),(so3.from_moment(x_r[9:12]),x_r[6:9])