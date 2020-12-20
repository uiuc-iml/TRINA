import numpy as np
import trimesh,os

def xyz_rpy_2_T(xyz,rpy):
    T=trimesh.transformations.euler_matrix(rpy[0],rpy[1],rpy[2])
    T[:3,3]=xyz
    return T

def T_2_xyz_rpy(T):
    xyz=T[:3,3].tolist()
    rpy=list(trimesh.transformations.euler_from_matrix(T[:3,:3]))
    return xyz,rpy

