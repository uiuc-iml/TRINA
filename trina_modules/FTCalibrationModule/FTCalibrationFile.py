from klampt.model.trajectory import Trajectory,SO3Trajectory
from klampt.io import loader
import autograd.numpy as np
from autograd import grad 
import scipy.optimize as opt
import scipy.linalg as lin


R_trajectory = loader.loadTrajectory('R_trajectory')
w_trajectory = loader.loadTrajectory('wrenches')

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
    m = np.array[0]
    cog = np.array[1:4]
    R_0 = to_matrix(Rs[0])
    G_0 = R_0.T@np.array([[0],[0],[-m*9.81]]) #gravity of the payload in EE frame, 2D numpy array
    Gravity = np.array([[0],[0],[-m*9.81]])
    tau_0 = np.cross(cog,G_0.ravel()) #torque in the EE frame
    w_0 = np.concatenate(G_0,tau_0)

    [m,n] = np.shape(Rs)
    error = 0.0
    for i in range(m):
        R = to_matrix(Rs[i]) #this is EE in global
        tmp = R.T@gravity 
        expected_f = tmp.ravel() - w_0[0:3]
        expected_tau = np.cross(cog,expected_f) - w_0[3:6]
        error += np.linalg.norm(np.concatenate(expected_f,expected_tau) - ws[i])

    return error

error_func_grad = grad(error_func)

def callback_func(x):
    return

x0 = np.zeros(4)
res = opt(fun = error_func,x0 = x0,jac = error_func_grad)#,callback = callback_func)

print(res)