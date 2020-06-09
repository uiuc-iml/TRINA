import open3d
from vision import PointCloud
import numpy as np


scene = open3d.io.read_point_cloud('scene.ply')

points = np.array(scene.points)

pcd = PointCloud(points)

pcd.removeNoise()

segments = pcd.segment_regionGrowing()

for i, segment in enumerate(segments):
    
    segment.save_to_ply(str(i) + '.ply')
