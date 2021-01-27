import time
import sys
import os
import argparse
from klampt.model.trajectory import SE3Trajectory, RobotTrajectory
from klampt.io import resource
from klampt import WorldModel
sys.path.append("../../../Motion")
import TRINAConfig


parser = argparse.ArgumentParser(description="Edit a path")
parser.add_argument("--file", type=str, default="",
	help="existing path to open")
args = parser.parse_args()

world = WorldModel()
world.readFile('../../../Motion/data/TRINA_world_cholera.xml')
robot = world.robot(0)

if args.file == "" or not os.path.exists(args.file):
	print("Starting from a default trajectory")
	home_config = TRINAConfig.get_klampt_model_q('cholera',
		left_limb=TRINAConfig.left_untucked_config,
		right_limb=TRINAConfig.right_untucked_config)
	traj = RobotTrajectory(robot=robot, times=[0,1], milestones=[home_config]*2)
else:
	print(f"Opening trajectory in file {args.file}")
	with open(args.file, "r") as tf:
		times = []
		milestones = []
		for line in tf.readlines():
			vals = line.split()
			times.append(float(vals[0]))
			milestones.append([float(m) for m in vals[2:]])
		traj = RobotTrajectory(robot=robot, times=times, milestones=milestones)

saved, result = resource.edit(
	"robot traj", traj, 'Trajectory',
	'Inertia Trajectory', world=world, referenceObject=robot)
