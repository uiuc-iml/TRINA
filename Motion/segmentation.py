from vision import PointCloud
import pcl
import cv2
import open3d as o3d
import numpy as np

# def plane_extraction(point_clouds):
#     point_clouds_array = np.asarray(point_clouds.points)
#     PC = PointCloud(point_clouds_array)
#     planes = PC.extract_plane()[0]
#     ptc_planes = []
#     for plane in planes:
#         cloud  = pcl.PointCloud()
#         cloud.points = plane.get_points()
#
#     return ptc_planes

def plane_extraction(point_clouds):
    point_clouds_array = np.asarray(point_clouds.points)
    PC = PointCloud(point_clouds_array)
    planes = PC.extract_plane()[0]
    ptc_planes = []
    for plane in planes:
        list = plane.get_points()
        ptc_planes.append(list)
    # ptc_planes = []
    # for plane in planes:
    #     cloud  = pcl.PointCloud()
    #     cloud.points = plane.get_points()
    ptc_planes = np.array(ptc_planes)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(ptc_planes)
    return pcd
