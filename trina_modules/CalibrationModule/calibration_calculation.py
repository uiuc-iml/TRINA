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
calidata_path = 'data/calibration_data/'


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

def calculation():
    """
    this is the main function of calibration calculation color.
    """
    joint_positions=[]
    data_file=open(calidata_path+'calibration_joint_positions.txt','r')
    for line in data_file:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        joint_positions.append(l2)
    data_file.close()

    #the marker transforms here are actually just points not transformations
    marker_transforms=[]
    data_file=open(calidata_path+'calibration_marker_transforms.txt','r')
    for line in data_file:
        line=line.rstrip()
        l=[num for num in line.split(' ')]
        l2=[float(num) for num in l]
        marker_transforms.append(l2)
    data_file.close()

    T_marker_assumed = loader.load('RigidTransform',calidata_path+'assumed_marker_world_transform.txt')
    T_marker_assumed = (so3.inv(T_marker_assumed[0]),T_marker_assumed[1])

    world=WorldModel()
    res=world.readFile("robot_model_data/ur5Blocks.xml")
    if not res:
        raise RuntimeError("unable to load model")
    #vis.add("world",world)
    robot=world.robot(0)
    link=robot.link(7)
    q0 = robot.getConfig()
    Tcameras = []
    Tlinks = []
    for i in range(len(joint_positions)):
        q = robot.getConfig()
        q[1:7] = joint_positions[i][0:6]
        robot.setConfig(q)
        Tlinks.append(link.getTransform())
        #vis.add("ghost"+str(i),q)
        #vis.setColor("ghost"+str(i),0,1,0,0.5)
    robot.setConfig(q0)
    #vis.add("MARKER",se3.mul(link.getTransform(),T_marker_assumed))
    #vis.setAttribute("MARKER","width",2)

    for loop in range(NLOOPS if ESTIMATE_MARKER_TRANSFORM else 1):
        ###Calculate Tcamera in EE (only needed for transform fit..)
        ###Tcameras = [se3.mul(se3.inv(Tl),se3.mul(T_marker_assumed,se3.inv(Tm_c))) for (Tl,Tm_c) in zip(Tlinks,marker_transforms)]
        
        #actually using only point fit here....
        if METHOD=='point fit':
            Tcamera2 = point_fit_transform([Tm_c for Tm_c in marker_transforms],[se3.mul(se3.inv(Tl),T_marker_assumed)[1] for Tl in Tlinks])
            Tcamera2 = [so3.from_matrix(Tcamera2[0]),Tcamera2[1]]
            Tcamera = Tcamera2
        #else:
        #    Tcamera = transform_average(Tcameras)
        #with the Tcamera from the current iteration, calculate marker points in world
        Tmarkers_world = [se3.apply(se3.mul(Tl,Tcamera),Tm_c) for (Tl,Tm_c) in zip(Tlinks,marker_transforms)] 
        #Tmarkers_link = [se3.mul(se3.inv(Tl),Tm) for Tl,Tm in zip(Tlinks,Tmarkers)]
        if ESTIMATE_MARKER_TRANSFORM:
            T_marker_assumed_inv = point_fit_transform([[0,0,0] for Tm_c in marker_transforms],[se3.apply(se3.mul(Tl,Tcamera2),Tm_c) for (Tl,Tm_c) in zip(Tlinks,marker_transforms)])
            T_marker_assumed_inv=[so3.from_matrix(T_marker_assumed_inv[0]),T_marker_assumed_inv[1]]
            #print T_marker_assumed_inv
            T_marker_assumed = T_marker_assumed_inv
            #print "New assumed marker position",T_marker_assumed
        else:
            pass
            #print "Would want new marker transform",transform_average(Tmarkers_world)
        #print "New estimated camera position",Tcamera
        SSE_t = 0
        for (Tm_c,Tl) in zip(marker_transforms,Tlinks):
            Tm_c_est = se3.mul(se3.mul(se3.inv(Tcamera),se3.inv(Tl)),T_marker_assumed)
            SSE_t += vectorops.distanceSquared(Tm_c,Tm_c_est[1])
        #print "RMSE rotation (radians)",np.sqrt(SSE_R/len(Tlinks))
        print "RMSE translations (meters)",np.sqrt(SSE_t/len(Tlinks))

    #Tcameras = [se3.mul(se3.inv(Tl),se3.mul(T_marker_assumed,se3.inv(Tm_c))) for (Tl,Tm_c) in zip(Tlinks,marker_transforms)]
    #Tmarkers = [se3.mul(Tl,se3.mul(Tcamera,Tm_c)) for (Tl,Tm_c) in zip(Tlinks,marker_transforms)]


    print "Saving to calibrated_camera_xform.txt"
    cali_txt_path = calidata_path+"calibrated_camera_xform.txt"
    loader.save(Tcamera,"RigidTransform",cali_txt_path)
    
    if ESTIMATE_MARKER_TRANSFORM:
        print "Saving to calibrated_marker_link_xform.txt"
        loader.save(T_marker_assumed,"RigidTransform",calidata_path+"calibrated_marker_world_xform.txt")
    #SSE_R = 0
    SSE_t = 0
    for (Tm_c,Tl) in zip(marker_transforms,Tlinks):
        Tm_c_est = se3.mul(se3.mul(se3.inv(Tcamera),se3.inv(Tl)),T_marker_assumed)
        SSE_t += vectorops.distanceSquared(Tm_c,Tm_c_est[1])
    print "RMSE translations (meters)",np.sqrt(SSE_t/len(Tlinks))

if __name__ == "__main__":
    calculation()