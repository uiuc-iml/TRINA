import time
import sys
from klampt.model.trajectory import SE3Trajectory, RobotTrajectory
from klampt.io import resource
from klampt import WorldModel
sys.path.append("/home/motion/TRINA/Motion")
import TRINAConfig


config = TRINAConfig.get_klampt_model_q('cholera',
	left_limb=TRINAConfig.left_untucked_config, 
	right_limb=TRINAConfig.right_untucked_config)

world = WorldModel()
world.readFile('/home/motion/TRINA/Motion/data/TRINA_world_cholera.xml')
robot = world.robot(0)

traj = RobotTrajectory(robot=robot, times=[0,1], milestones=[config]*2)

saved, result = resource.edit(
	"robot traj", traj, 'Trajectory', 
	'Inertia Trajectory', world=world, referenceObject=robot)
