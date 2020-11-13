import klampt
from klampt.math import vectorops,so3,se3
import numpy as np
from klampt import WorldModel,vis
from utils import *
def URDFCalibration(Tl,ql,Tr,qr,T_marker_1,T_c_l,T_c_r,world_path,URDF_save_path,links):
    """
    Optimize for the transforms of thea arms and the wrist-mounted cameras

    Since the initial guess would be close, use local solve only for now
    """
   
    #first load the initial guess for the arm transforms
    world = WorldModel()
    res = world.readFile(world_path)
    if not res:
        logger.error('unable to load model')
        raise RuntimeError("unable to load model")

    robot_model = world.robot(0)
    #TODO: check the link number
    link_base_l = robot_model.link(15)
    link_base_r = robot_model.link(35)
    T_l_0 = link_base_l.getTransform()
    T_r_0 = link_base_r.getTransform()


    def funcl(x):
        """
        x: [0:6] pos + moment of shoulder [6:12] pos + moment of camera
        """
        expr=0.0
        for T_c,q in zip(Tl,ql)):
            T_EE = getLeftEETransform(robot_model,q)
            T_EE_base = se3.mul(se3.inv(T_l_0),T_EE) #EE in arm base
            error = vectorops.normSquared(se3.error(T_marker_1,se3.mul(T_EE,T_c)))

            EERef=se3.apply(l2c,p)
            EEDiff=vectorops.sub(EERef,EE)
            expr+=vectorops.dot(EEDiff,EEDiff)
        for EE,p in zip(EEsRight,psRight):
            if isinstance(EE,tuple):
                EE=se3.apply(EE,rPos)
            EERef=se3.apply(r2c,p)
            EEDiff=vectorops.sub(EERef,EE)
            expr+=vectorops.dot(EEDiff,EEDiff)
        expr=math.sqrt(expr/(len(psLeft)+len(psRight)))
        if callback:
            print("Local solve: func(arm)=%f"%expr)
        return expr


    import scipy.optimize as sopt
    x=sopt.minimize(func,x,method='L-BFGS-B').x
    print("Local solve: func(arm)=%f"%func(x))
    l2c,r2c,lPos,rPos=extract_se3(x,'TTtt')
    return l2c,r2c,lPos,rPos,func(x)

    return Tl,Tr