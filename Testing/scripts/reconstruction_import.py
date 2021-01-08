import rospy
from std_msgs.msg import String
from shape_msgs import Mesh
from klampt.io.ros import from_Mesh
from klampt import WorldModel,Geometry3D
from klampt import vis
import tf 
trina_world_path = '/home/motion/TRINA/Motion/data/TRINA_world_anthrax_PointClick.xml'

global world

world = WorldModel(trina_world_path)
vis.init('PyQt')
vis.add('world',world)
vis.show()

import sys
print(sys.executable)

def callback(data):
    print(type(data))
    a = from_Mesh(data)
    vis.add('mesh',a)
    print(type(a))

    pass

rospy.init_node('mesh_listener', anonymous = True)
rospy.Subscriber('mesh',Mesh,callback)

rospy.spin()
