from klampt import *
from klampt.vis import editors
from klampt.model import sensing

w = WorldModel()
res = w.readFile("../Models/simulation_worlds/TRINA_world_bubonic_with_obstacle.xml")
assert res,"Couldn't read file?"
s = Simulator(w)
cam = s.controller(0).sensor('realsense_right')

editors.run(editors.SensorEditor("realsense_right",cam,"The right Realsense camera",w))
lidar = s.controller(0).sensor('lidar')


