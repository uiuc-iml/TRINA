from klampt.model.trajectory import SE3Trajectory
from klampt.io import resource
from klampt import WorldModel

world = WorldModel()
world.readFile('/home/motion/TRINA/Motion/data/TRINA_world_bubonic.xml')
robot = world.robot(0)

saved, result = resource.edit("robot traj", [], 'Trajectory', 'Inertia Trajectory', world=world, referenceObject=robot)
