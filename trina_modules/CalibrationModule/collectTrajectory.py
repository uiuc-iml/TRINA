"""
For ease of testing, use motion client/server instead of Jarvis.

Note:
When the robot is started, the gripper should already be attached
"""
import sys
sys.path.append('../../Motion/')
from motion_client import MotionClient
from klampt.io import loader
from copy import copy,deepcopy
import TRINAConfig
from klampt.model.trajectory import SO3Trajectory,Trajectory
import time
import math
#Start the robot
robot = MotionClient(address = 'http://localhost:8080')
robot.startServer(mode = 'Physical',components = ['left_limb'],codename = 'anthrax')
robot.startup()


initial_config = copy(TRINAConfig.left_untucked_config)
robot.setLeftLimbPositionLinear(initial_config,5)
time.sleep(5)
print('Robot Moved to Initial Config, start collecting data')

def desired_w(t):
    assert t >= 0, "time should be positive"
    m1 = 10.0
    m2 = 20.0
    if t <= m1:
        #return [0,-math.pi/3.0/m1,-math.pi/3.0/m1]
        return [0,-math.pi/3.0/m1,0]
    elif t> m1 and t<= m2:
        return [-math.pi/3.0/(m2-m1),math.pi/3.0/(m2-m1),0]
    else:
        return [0,0,0]


current_time = 0
dt = 0.02
total_time = 10.0
R_history = []
t_history = []
wrench_history = []
wrench_global_history = []
while current_time <= total_time:

    if current_time > 1:
        (R,_) = robot.sensedLeftEETransform()
        wrench_global = robot.sensedLeftEEWrench(frame = 'global',format = 'raw')
        wrench = robot.sensedLeftEEWrench(frame = 'local', format = 'raw')
        R_history.append(R)
        t_history.append(current_time)
        wrench_global_history.append(wrench_global)
        wrench_history.append(wrench)
        w = desired_w(current_time)
        robot.setLeftEEVelocity([0,0,0]+w,tool = [0,0,0.0])
    current_time += dt
    time.sleep(dt)

trajectory = SO3Trajectory(times = t_history,milestones = R_history)
wrenches = Trajectory(times = t_history,milestones = wrench_history)
wrenches_global = Trajectory(times = t_history,milestones = wrench_global_history)

loader.save(trajectory,'auto','R_trajectory')
loader.save(wrenches,'auto','wrenches_no_weight')
loader.save(wrenches_global,'auto','wrenches_global_no_weight')

robot.setLeftLimbPositionLinear(initial_config,5)
time.sleep(5)
robot.shutdown()
