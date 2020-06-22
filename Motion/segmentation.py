from vision import PointCloud
import pcl
import cv2
import open3d
import numpy as np

def plane_extraction(point_clouds):
    point_clouds_array = np.asarray(point_clouds.point)
    PC = PointCloud(point_clouds_array)
    return PC.extract_plane()
