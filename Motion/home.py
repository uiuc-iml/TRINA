from motion import *
from klampt.math import vectorops,so3
import math
import time

robot = Motion(mode = 'Physical')
robot.startup()
time.sleep(0.2)
print('Robot start() called')
leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)
rightDownConfig = [-0.6860278288470667, -2.182815214196676, 2.0740626494037073, -0.7054613393596192, 1.5997309684753418, -1.4906814734088343]
forwardConfig = [-0.353679424536684, -2.5668469504059237, -0.7772500110039505, 3.3443770762889766, -1.1130922959204652, 0.09739075748121706]
rotatedConfig = [-0.35670282705574874, -2.5724731944194756, -0.7598447478490323, 3.3048581414264664, -1.1410026546777612, 1.6178811981635732]
upConfig = [-0.44262933789081105, -2.5868874513094635, -0.9121955044497172, 3.4725910883710718, -1.2268820110866854, 1.6153649982327345]
downConfig = [-0.27958604012970756, -2.6317944630573606, -0.4794259718505932, 3.082770718089447, -1.0639381628492601, 1.6202528544627122]

##move to untucked...
startTime = time.time()
#print(robot.sensedRightLimbPosition())
#robot.setRightLimbPositionLinear(rightDownConfig,5)
robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
time.sleep(5)

robot.shutdown()