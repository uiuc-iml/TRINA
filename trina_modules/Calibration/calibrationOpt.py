import klampt
from klampt.math import vectorops,so3,se3
from klampt.io import loader
import numpy as np
from klampt import WorldModel,vis
from utils_my import *
import math
from scipy.spatial.transform import Rotation as ScipyR

def klampt2Scipy(R):
    return ScipyR.from_matrix([[R[0], R[3], R[6]],\
                   [R[1], R[4], R[7]],\
                   [R[2], R[5], R[8]]])

def se3_error(T1,T2): ##somehow klampt se3.error gives issues when converting from R to rot vector:assert(z2>-1e-5)
    R_error = klampt2Scipy(so3.mul(T1[0],so3.inv(T2[0]))).as_rotvec().tolist()
    t_error = vectorops.sub(T1[1],T2[1])

    return R_error + t_error



def URDFCalibration(Tl,ql,Tr,qr,T_marker_1,T_c_l_0,T_c_r_0,robot_path,links):
    """
    Optimize for the transforms of thea arms and the wrist-mounted cameras

    Since the initial guess would be close, use local solve only for now
    """
   
    #first load the initial guess for the arm transforms
    world = WorldModel()
    res = world.loadElement(robot_path)
    robot_model = world.robot(0)
    link_base_l = robot_model.link(10)
    link_base_r = robot_model.link(18)
    T_l_0 = link_base_l.getTransform()
    T_r_0 = link_base_r.getTransform()
    x_l_0 = T_l_0[1] + so3.moment(T_l_0[0]) + T_c_l_0[1] + so3.moment(T_c_l_0[0])
    x_r_0 = T_r_0[1] + so3.moment(T_r_0[0]) + T_c_r_0[1] + so3.moment(T_c_r_0[0])

    def funcl(x):
        """
        x: [0:6] pos + moment of shoulder [6:12] pos + moment of camera
        """
        x = x.tolist()
        T_base = (so3.from_moment(x[3:6]),x[0:3])
        T_c_EE = (so3.from_moment(x[9:12]),x[6:9])
        trans_error = 0.
        angle_error = 0.
        for T_m_c,q in zip(Tl,ql):
            
            #transform Tl..
            T_EE = getLeftLinkTransform(robot_model,q,links[0],model = 'cholera')
            T_EE_base = se3.mul(se3.inv(T_l_0),T_EE) #EE in arm base
            T_m_predicted = se3.mul(se3.mul(se3.mul(T_base,T_EE_base),T_c_EE),T_m_c)
            #only use position
            error_vec = vectorops.sub(T_marker_1[1],T_m_predicted[1])
            trans_error += vectorops.normSquared(error_vec)
            #use entire transform
            # error_vec = se3.error(T_marker_1,T_m_predicted)
            # trans_error += vectorops.normSquared(error_vec[0:3]) 
            # angle_error += vectorops.normSquared(error_vec[3:6])
        expr = math.sqrt(trans_error/len(Tl))+ 1.0*math.sqrt(angle_error/len(Tl))
        print("Local solve: func(left arm)=%f"%expr)
        print('pos error:',math.sqrt(trans_error/(len(Tl))),'angle error', math.sqrt(angle_error/(len(Tl))))
        
        return expr

    def funcr(x):
        """
        x: [0:6] pos + moment of shoulder [6:12] pos + moment of camera
        """
        x = x.tolist()
        T_base = (so3.from_moment(x[3:6]),x[0:3])
        T_c_EE = (so3.from_moment(x[9:12]),x[6:9])
        trans_error = 0.
        angle_error = 0.
        for T_m_c,q in zip(Tr,qr):
            T_EE = getRightLinkTransform(robot_model,q,links[1],model = 'cholera')
            T_EE_base = se3.mul(se3.inv(T_r_0),T_EE) #EE in arm base
            T_m_predicted = se3.mul(se3.mul(se3.mul(T_base,T_EE_base),T_c_EE),T_m_c)
            #only use position
            # error_vec = vectorops.sub(T_marker_1[1],T_m_predicted[1])
            # trans_error += vectorops.normSquared(error_vec)
            #use entire transform
            error_vec = se3.error(T_marker_1,T_m_predicted)
            trans_error += vectorops.normSquared(error_vec[0:3]) 
            angle_error += vectorops.normSquared(error_vec[3:6])
        expr = math.sqrt(trans_error/len(Tr)) + 1*math.sqrt(angle_error/len(Tr))
        print('pos error:',math.sqrt(trans_error/len(Tr)),'angle error', math.sqrt(angle_error/len(Tr)))
        return expr



    import scipy.optimize as sopt

    bnds = ((-0.03, 0.03), (0.13, 0.23),(1.23,1.7),(None,None),(None,None),(None,None),(-0.2,0.2),(-0.2,0.2),(-0.2,0.2),(None,None),(None,None),(None,None))
    res = sopt.minimize(funcl,np.array(x_l_0),bounds = bnds,method='L-BFGS-B')
    x_l = res.x
    bnds = ((-0.03, 0.03), (-0.23, -0.13),(1.23,1.7),(None,None),(None,None),(None,None),(-0.2,0.2),(-0.05,0),(-0.2,0.2),(None,None),(None,None),(None,None))
    res = sopt.minimize(funcr,np.array(x_r_0),bounds = bnds,method='L-BFGS-B')
    x_r = res.x

    print("Left Arm Local solve: func(left arm)=%f"%funcl(x_l))
    print('initial left shoulder transform:',T_l_0)
    print('final left shoulder transform:',(so3.from_moment(x_l[3:6]),x_l[0:3]))
    print('initial camera transform:',T_c_l_0)
    print('final camera transform:',(so3.from_moment(x_l[9:12]),x_l[6:9]))

    print("Right Arm Local solve: func(left arm)=%f"%funcr(x_r))
    print('initial Right shoulder transform:',T_r_0)
    print('final right shoulder transform:',(so3.from_moment(x_r[3:6]),x_r[0:3]))
    print('initial camera transform:',T_c_r_0)
    print('final camera transform:',(so3.from_moment(x_r[9:12]),x_r[6:9]))


    #left shoulder, left camera, right shoulder, right camera
    return (so3.from_moment(x_l[3:6]),x_l[0:3].tolist()),(so3.from_moment(x_l[9:12]),x_l[6:9].tolist()),\
        (so3.from_moment(x_r[3:6]),x_r[0:3].tolist()),(so3.from_moment(x_r[9:12]),x_r[6:9].tolist())

def StaticCalibration(T_0,qs,cameras,T_marker_0,world_path,link):

    """
    Optimize for the transforms of the SLAM cameras (realsense torso, zed torso)
    """
   
    #first load the initial guess for the arm transforms
    world = WorldModel()
    res = world.readFile(world_path)
    if not res:
        raise RuntimeError("unable to load model")

    robot_model = world.robot(0)
    link_base_l = robot_model.link(link)
    N_of_cameras = len(cameras)
    p_marker = T_marker_0[1]

    def func(x):
        """
        x: [0:6] pos + moment of marker in link local frame
        [6:12] first camera transform ..
        """
        x = x.tolist()
        T_base = (so3.from_moment(x[3:6]),x[0:3])
        T_c_EE = (so3.from_moment(x[9:12]),x[6:9])
        counter = 0
        trans_error = 0.
        angle_error = 0.
        for T_m_c,q in zip(Tl,ql):
            #transform Tl..
            T_EE = getLeftLinkTransform(robot_model,q,links[0],model='cholera')
            T_EE_base = se3.mul(se3.inv(T_l_0),T_EE) #EE in arm base
            T_m_predicted = se3.mul(se3.mul(se3.mul(T_base,T_EE_base),T_c_EE),T_m_c)

            error_vec = se3.error(T_marker_1,T_m_predicted)
            trans_error += vectorops.normSquared(error_vec[0:3]) 
            angle_error += vectorops.normSquared(error_vec[3:6])
        expr = math.sqrt(trans_error/len(Tl)) + 0.001*math.sqrt(angle_error/len(Tl))
        # print("Local solve: func(left arm)=%f"%expr)
        print('pos error:',math.sqrt(trans_error/(54)),'angle error', math.sqrt(angle_error/(54)))
        return expr



    import scipy.optimize as sopt

    bnds = ((-0.015, 0.025), (0.13, 0.23),(1.2,1.8),(None,None),(None,None),(None,None),(-0.2,0.2),(-0.2,0.2),(-0.2,0.2),(None,None),(None,None),(None,None))
    res = sopt.minimize(funcl,np.array(x_l_0),bounds = bnds,method='L-BFGS-B')
    x_l = res.x


    #left shoulder, left camera, right shoulder, right camera
    return (so3.from_moment(x_l[3:6]),x_l[0:3]),(so3.from_moment(x_l[9:12]),x_l[6:9])





##Zherong's optimization code 
# def rotation_vec(R,res):
#     return [-math.pi+math.pi*2*i/(res-1) for i in R]

# def rotation(R,res):
#     return so3.from_rotation_vector(rotation_vec(R,res))

# def sos2(M,func,res):
#     import gurobipy as gp
#     vars=[0. for i in range(res)]
#     for x in range(res):
#         for y in range(res):
#             for z in range(res):
#                 vars[z]+=func(x,y,z)
#     varsTmp=[M.addVar(lb=0.,ub=1.) for i in range(res)]
#     for i in range(res):
#         M.addConstr(vars[i]==varsTmp[i])
#     M.addSOS(gp.GRB.SOS_TYPE2,varsTmp)
#     M.addConstr(sum(varsTmp)==1)
#     return vars

# def rotation_constraint(rot,M,res):
#     rotC=[0. for i in range(9)]
#     rotCNode=[[[rotation([x,y,z],res) for x in range(res)] for y in range(res)] for z in range(res)]
#     rotCWeight=[[[M.addVar(ub=1) for x in range(res)] for y in range(res)] for z in range(res)]
#     for x in range(res):
#         for y in range(res):
#             for z in range(res):
#                 rotC=vectorops.add(rotC,vectorops.mul(rotCNode[x][y][z],rotCWeight[x][y][z]))
#     for i in range(9):
#         M.addConstr(rotC[i]==rot[i])
#     def funcx(x,y,z):
#         return rotCWeight[x][y][z]
#     xVar=sos2(M,funcx,res)
#     def funcy(x,y,z):
#         return rotCWeight[x][z][y]
#     yVar=sos2(M,funcy,res)
#     def funcz(x,y,z):
#         return rotCWeight[z][x][y]
#     zVar=sos2(M,funcz,res)
#     return xVar,yVar,zVar,rotCWeight

# import gurobipy as gp
# #declare model, variable
# M=gp.Model("Calibration")
# M.setParam("LogToFile","")
# M.setParam("LogToConsole",1)
# T_r_0=([M.addVar(lb=-1,ub=1) for i in range(9)],[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)])
# # r2c=([M.addVar(lb=-1,ub=1) for i in range(9)],[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)])
# lPos=[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)]
# # rPos=[M.addVar(lb=-gp.GRB.INFINITY) for i in range(3)]

# #rotation constraint
# lRot=rotation_constraint(l2c[0],M,res)
# # rRot=rotation_constraint(r2c[0],M,res)

# #energy
# expr=0.
# for T_m_c,q in zip(Tl,ql):
#     T_EE = getLeftLinkTransform(robot_model,q,links[0])
#     T_EE_base = se3.mul(se3.inv(T_l_0),T_EE) #EE in arm base
#     T_m_predicted = se3.mul(se3.mul(se3.mul(T_base,T_EE_base),T_c_EE),T_m_c)
#     error = vectorops.normSquared(se3.error(T_marker_1,T_m_predicted))
#     expr += error
# expr = math.sqrt(expr/(len(Tl)))
# print("Local solve: func(left arm)=%f"%expr)
# # for EE,p in zip(EEsRight,psRight):
# #     if isinstance(EE,tuple):
# #         EE=se3.apply(EE,rPos)
# #     EERef=se3.apply(r2c,p)
# #     EEDiff=vectorops.sub(EERef,EE)
# #     expr+=vectorops.dot(EEDiff,EEDiff)
# M.setObjective(expr,gp.GRB.MINIMIZE)

# #solve
# M.optimize()
