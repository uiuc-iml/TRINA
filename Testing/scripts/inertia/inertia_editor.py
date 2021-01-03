import time
import sys
from klampt.model.trajectory import SE3Trajectory, RobotTrajectory
from klampt.io import resource
from klampt import WorldModel
sys.path.append("/home/motion/TRINA")
from Motion import MotionClient

m = MotionClient('http://127.0.0.1:8080')
m.startServer(mode = "Physical", components = ['right_limb','left_limb'], codename = 'bubonic')
m.startup()
time.sleep(0.05)
config = m.getKlamptSensedPosition()
time.sleep(0.05)
m.shutdown()

world = WorldModel()
world.readFile('/home/motion/TRINA/Motion/data/TRINA_world_bubonic.xml')
robot = world.robot(0)

traj = RobotTrajectory(robot=robot, times=[0,1], milestones=[config]*2)

saved, result = resource.edit(
	"robot traj", traj, 'Trajectory', 
	'Inertia Trajectory', world=world, referenceObject=robot)
