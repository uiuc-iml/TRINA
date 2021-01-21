import klampt
from klampt import vis
from klampt import PointCloud
from klampt import WorldModel
from klampt.io import open3d_convert
import open3d as o3d

pcd = o3d.io.read_point_cloud("../../test_grasping.pcd")
klampt_pcd = open3d_convert.from_open3d(pcd)

vis.debug(klampt_pcd)
