from klampt.model.trajectory import SE3Trajectory, RobotTrajectory
from klampt.io import resource
from klampt import WorldModel

world = WorldModel()
world.readFile('/home/motion/TRINA/Motion/data/TRINA_world_bubonic.xml')
robot = world.robot(0)

traj = RobotTrajectory(robot=robot, times=[0,1], milestones=[[0]*22]*2)

saved, result = resource.edit(
	"robot traj", traj, 'Trajectory', 
	'Inertia Trajectory', world=world, referenceObject=robot)
