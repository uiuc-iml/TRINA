from motion import *
from klampt.math import vectorops,so3
import math
import time

##### testing using the kinematic mode..############
#
###################################################
robot = Motion(mode = 'Kinematic')
robot.startup()
print('Robot start() called')
leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)

world = robot.getWorld()
vis.add("world",world)
vis.show()
robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
startTime = time.time()
while (time.time()-startTime < 5):
    vis.lock()
    #robot.setBaseVelocity([0.5,0.1])
    vis.unlock()
    time.sleep(0.02)

    #print(time.time()-startTime)

##use EE velocity drive
startTime = time.time()
robot.setLeftEEVelocity(v = [0.0,0,0], w = [0,0,0.2],tool = [0.1,0,0])
while (time.time()-startTime < 5):
    vis.lock()
    #robot.setBaseVelocity([0.5,0.1])
    vis.unlock()
    time.sleep(0.02)
    if robot.cartesian_drive_fail():
    	break
    	print(time.time()-startTime)

# for i in range(5):
# 	print("########################")
# robot.setLeftEEVelocity(v = [-0.01,0,0], w =[0,0,0] , tool = [0,0,0])
# startTime = time.time()
# while (time.time()-startTime < 5):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     if robot.cartesian_drive_fail():
#     	break
#     	print(time.time()-startTime)
vis.kill()
print('flag')
robot.shutdown()




###############Physical starts here #############
# 
# This is for the handover and shaking hand demo
#################################################
# robot = Motion(mode = 'Physical')
# robot.startup()
# print('Robot start() called')
# leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
# leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
# rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
# rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)
# rightDownConfig = [-0.6860278288470667, -2.182815214196676, 2.0740626494037073, -0.7054613393596192, 1.5997309684753418, -1.4906814734088343]



# #move to untucked...
# startTime = time.time()
# # robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
# robot.setRightLimbPositionLinear(rightDownConfig,5)
# robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
# time.sleep(5)

# print('3')
# time.sleep(1)
# print('2')
# time.sleep(1)
# print('1')
# time.sleep(1)
# ##move forward
# currentLeftEETransform = robot.sensedLeftEETransform()
# targetLeftEETransform = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0.2,0,0.1]))
# robot.setLeftEEInertialTransform(targetLeftEETransform,2)
# time.sleep(4)

# ##rotate 90 degrees
# currentLeftEETransform = robot.sensedLeftEETransform()
# targetLeftEETransform = (so3.mul(so3.from_moment([math.pi/2.0,0,0]),currentLeftEETransform[0]),currentLeftEETransform[1])
# robot.setLeftEEInertialTransform(targetLeftEETransform,2.5)
# time.sleep(2.5)

# ##shake hand up and down
# currentLeftEETransform = robot.sensedLeftEETransform()
# targetLeftEETransformUp = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0,0,0.1]))
# targetLeftEETransformDown = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0,0,-0.06]))

# robot.setLeftEEInertialTransform(targetLeftEETransformUp,1)
# time.sleep(1.1)

# robot.setLeftEEInertialTransform(targetLeftEETransformDown,1.6)
# time.sleep(1.7)
# robot.setLeftEEInertialTransform(targetLeftEETransformUp,1.6)
# time.sleep(1.7)
# robot.setLeftEEInertialTransform(targetLeftEETransformDown,1.6)
# time.sleep(1.7)
# robot.setLeftEEInertialTransform(targetLeftEETransformUp,1.6)
# time.sleep(1.7)
# robot.setLeftEEInertialTransform(targetLeftEETransformDown,1.6)
# time.sleep(1.7)
# print("shutting down")
# robot.shutdown()
# print("program ended")

