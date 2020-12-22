import klampt
from klampt import vis
from klampt import PointCloud
from klampt import WorldModel
from klampt.io import open3d_convert
import open3d as o3d

pcd = o3d.io.read_point_cloud("/home/motion/item_set/item1-68.pcd")
klampt_pcd = open3d_convert.from_open3d(pcd)

pcd2 = open3d_convert.to_open3d(klampt_pcd)
klampt_pcd = open3d_convert.from_open3d(pcd2)


world = klampt.WorldModel()

vis.debug(klampt_pcd)
