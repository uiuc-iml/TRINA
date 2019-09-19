#RobotController.py
from ur5_new import Ur5Controller
import sys
from klampt import *
from klampt import vis
from klampt.vis import GLSimulationPlugin
from klampt.math import vectorops
from klampt.math import se3
from klampt.math import so3
from klampt.model import ik
import time


from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
parser = ArgumentParser(description='PyUniversalRobot wiggle test', formatter_class=ArgumentDefaultsHelpFormatter)
parser.add_argument('robot', help='robot IP address')
parser.add_argument('--frequency', '-f', type=float, help='wiggle frequency (Hz)', default=0.1)
parser.add_argument('--amplitude', '-a', type=float, help='wiggle amplitude (deg)', default=5)
args = parser.parse_args()
#ctrl = Ur5Controller(args.robot, frequency=args.frequency, amplitude=args.amplitude)
ctrl = Ur5Controller(args.robot, speed=3, mode='discrete', gripper=0)
ctrl.start()
# ctrl.setMode('discrete')
# ctrl.setSpeed(6)


zOffset=1.12395

world=WorldModel()
res=world.readFile('/home/motion/iml-internal/Hardware/ur5Cart_no_gripper.xml')
if not res:
    raise RuntimeError("unable to load model")
robot=world.robot(0)


pi=3.14159265359
link=robot.link(7)
robot.setConfig([0,147.52*pi/180.0,-17.36*pi/180.0,127.84*pi/180.0,-201.77*pi/180.0,-92.27*pi/180.0,4.27*pi/180.0,0])
#6 config

#x = 0.4, y=-0.2 z=0
#forward_link6 = [1,0,0]
#perp_link6 = [0,1,0]

start_target = [0.4, -0.2, zOffset]
end_target = [0.4, 0.2, zOffset]
num_milestones = 20.0

ik_good = 1
total_path = []
activeDoFs=[0,1,2,3,4,5,6]
for i in range(20):
    target = vectorops.madd(start_target, vectorops.sub(end_target, start_target), i*1.0/num_milestones)
    obj=ik.objective(link,local=[[0,0,0],[1,0,0],[0,1,0]],world=[vectorops.add(target,[0,0,0]),vectorops.add(target,[0,0,-1]),vectorops.add(target,[0,1,0])])
    seed=robot.getConfig()
    s=IKSolver(robot)
    s.add(obj)
    s.setTolerence=0.0005
    s.setActiveDofs(activeDoFs)
    s.setBiasConfig(seed)
    res=s.solve()
    if not res:
        ik_good = 0
        print "IK Failure on ", target, " i =", i
    config=robot.getConfig()
    ur5config = config[1:7]
    a = 1
    if i >= num_milestones/2:
        a = 0
    ur5config.append(a)
    total_path.append(ur5config)
    print vectorops.mul(config,1)

'''
ctrl.setPath([0,0,0,0,0,0,0])
default_milestone = [0,0,0,0,0,0,0]
for i in range(6):
    default_milestone[i] = default_milestone[i] + math.pi/4
    # if default_milestone[-1]:
    #     default_milestone[-1] = 0
    # else:
    #     default_milestone[-1] = 1
    ctrl.appendPath([default_milestone])
ctrl.appendPath([0,0,0,0,0,0,1])
ctrl.run()
'''

if ik_good:
   ctrl.setPath(total_path)
   ctrl.run()

while 1:
    try:
        time.sleep(0.1)
    except KeyboardInterrupt:
        ctrl.stop()
        break

#signal.signal(signal.SIGINT, ctrl.stop)
print('done')
