from klampt.model.trajectory import Trajectory,SO3Trajectory
from klampt.io import loader
import autograd.numpy as np
from autograd import grad
import scipy.optimize as opt
import scipy.linalg as lin
from math import sqrt

R_trajectory = loader.loadTrajectory('R_trajectory')
w_trajectory = loader.loadTrajectory('wrenches')
R_base_global_left = np.array([[sqrt(0.5),-sqrt(0.5),0],\
                    [sqrt(0.25),sqrt(0.25),-sqrt(0.5),],\
                    [sqrt(0.25),sqrt(0.25),sqrt(0.5)]])##base in global
R_global_base_left = R_base_global_left.T
global Rs,ws
Rs = np.array(R_trajectory.milestones)
ws = np.array(w_trajectory.milestones)


def to_matrix(a):
    return np.array([[a[0],a[3],a[6]],[a[1],a[4],a[7]],[a[2],a[5],a[8]]])
#define the cost function and cost function gradient
def error_func(x):
    """
    x is [m,cog]
    """
    global Rs,ws
    #calculate the offset due to zeroing the sensor at the beginning
    m = x[0]
    cog = x[1:4]
    R_0 = to_matrix(Rs[0]) ##EE in R_global_base_left
    gravity = np.array([[0],[0],[-m*9.81]])
    G_0_base = np.dot(R_global_base_left,np.array([0,0,-m*9.81])) #this is irrelevant to EE orientation
    R_base_EE = np.dot(R_0.T,R_base_global_left)
    G_0_EE = np.dot(R_base_EE,G_0_base)
    tau_0_EE = np.cross(cog,G_0_EE) #torque in the EE frame
    #tau_0_base = np.dot(R_base_EE.T,tau_0_EE)
    offset_f_EE = ws[0,0:3] - G_0_EE  #offset in the EE frame
    offset_tau_EE = ws[0,3:6] - tau_0_EE
    #offset in the EE frame
    offset = np.concatenate((offset_f_EE,offset_tau_EE))


    [N_of_pts,n] = np.shape(Rs)
    error = 0.0

    for i in range(N_of_pts):
        R = to_matrix(Rs[i]) #this is EE in global
        G_base = np.dot(R_global_base_left,np.array([0,0,-m*9.81]))
        R_base_EE = np.dot(R.T,R_base_global_left)
        G_EE = np.dot(R_base_EE,G_base)
        tau_EE = np.cross(cog,G_EE) #torque in the EE frame
        expected_f = G_EE + offset[0:3]
        expected_tau = tau_EE + offset[3:6]
        error += np.linalg.norm(np.concatenate((expected_f,expected_tau)) - ws[i])

        #print(expected_f,ws[i,0:3],G_EE)
        #print(expected_f,expected_f - ws[i,0:3])

    return error

error_func_grad = grad(error_func)

def callback_func(x):
    return

print(error_func(np.array([-1.6,-0.02,0,0])))
x0 = np.array([-1.8,0.05,0,0])# np.zeros(4)
res = opt.minimize(fun = error_func,x0 = x0,jac = error_func_grad)#,callback = callback_func)
#
print(res)
