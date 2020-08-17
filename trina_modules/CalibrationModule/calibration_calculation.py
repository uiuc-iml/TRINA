import sys
from klampt import *
from klampt.math import se3,so3,vectorops
from klampt.io import loader
#from klampt import vis
import time
import numpy as np

#if true, estimates the marker transform in the optimization loop
ESTIMATE_MARKER_TRANSFORM = True
NLOOPS = 1000
#point-fit is more robust to marker orientation estimation errors
METHOD = 'point fit'
#METHOD = 'transform fit'


def transform_average(Ts):
    t = vectorops.div(vectorops.add(*[T[1] for T in Ts]),len(Ts))
    qs = [so3.quaternion(T[0]) for T in Ts]
    q = vectorops.div(vectorops.add(*qs),len(Ts))
    return (so3.from_quaternion(q),t)

def point_fit_rotation(a,b):
    assert len(a)==len(b)
    A = np.array(a).T
    B = np.array(b).T
    BAt = np.dot(B,A.T)
    U,W,Vt = np.linalg.svd(BAt)
    R = np.dot(U,Vt)
    return R

def point_fit_transform(a,b):
    assert len(a)==len(b)
    A = np.array(a).T
    B = np.array(b).T
    amean = np.average(A,axis=1)
    bmean = np.average(B,axis=1)
    A = A - np.column_stack([amean]*len(a))
    B = B - np.column_stack([bmean]*len(b))
    BAt = np.dot(B,A.T)
    U,W,Vt = np.linalg.svd(BAt)
    R = np.dot(U,Vt)
    return R,bmean-np.dot(R,amean)

def calculation(pts,EE_transforms,camera_guess,marker_guess):
    """This is the function of running optimization for calibration
    
    Parameters:
    ----------------
    pts: a list of lists of three floats, the positions of the marker center in camera frame
    EE_transforms: a list of Klampt rigid transforms, transforms of the EE when taking the pictures, in global frame
    camera_guess: rigid transform, the initial guess of the camera transform w.r.t. EE
    marker_guess: rigid transform, the initial guess of the marker transform in the global frame
    """
    #the marker transforms here are actually just points not transformations
    marker_transforms = pts
    T_marker_assumed = marker_guess
    T_EE = EE_transforms
    for loop in range(NLOOPS if ESTIMATE_MARKER_TRANSFORM else 1):
        #Only using point fit, since we only have pts not transforms
        #This estimates Tcamera while assuming some marker transform
        if METHOD=='point fit':
            Tcamera2 = point_fit_transform([Tm_c for Tm_c in marker_transforms],[se3.mul(se3.inv(Tl),T_marker_assumed)[1] for Tl in T_EE])
            Tcamera2 = [so3.from_matrix(Tcamera2[0]),Tcamera2[1]]
            Tcamera = Tcamera2
        #with the Tcamera from the current iteration, calculate marker points in world
        Tmarkers_world = [se3.apply(se3.mul(Tl,Tcamera),Tm_c) for (Tl,Tm_c) in zip(T_EE,marker_transforms)] 
        #Tmarkers_link = [se3.mul(se3.inv(Tl),Tm) for Tl,Tm in zip(T_EE,Tmarkers)]

        #Here estimate Tmarker, while keeping camera transform fixed
        if ESTIMATE_MARKER_TRANSFORM:
            T_marker_assumed_inv = point_fit_transform([[0,0,0] for Tm_c in marker_transforms],[se3.apply(se3.mul(Tl,Tcamera2),Tm_c) for (Tl,Tm_c) in zip(T_EE,marker_transforms)])
            T_marker_assumed_inv=[so3.from_matrix(T_marker_assumed_inv[0]),T_marker_assumed_inv[1]]
            T_marker_assumed = T_marker_assumed_inv

        SSE_t = 0
        for (Tm_c,Tl) in zip(marker_transforms,T_EE):
            Tm_c_est = se3.mul(se3.mul(se3.inv(Tcamera),se3.inv(Tl)),T_marker_assumed)
            SSE_t += vectorops.distanceSquared(Tm_c,Tm_c_est[1])
        print("RMSE translations (meters)",np.sqrt(SSE_t/len(T_EE)))
    
    return Tcamera,T_marker_assumed
    

if __name__ == "__main__":
    calculation()