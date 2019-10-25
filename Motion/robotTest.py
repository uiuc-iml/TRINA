from motion import *
from klampt.math import vectorops,so3
import math
import time
# robot = Motion(mode = 'Kinematic')
# robot.startup()
# print('Robot start() called')
# leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
# leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
# rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
# rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)


##move to untucked...
# startTime = time.time()
# world = robot.getWorld()
# vis.add("world",world)
# vis.show()
# robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
# robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
# while (time.time()-startTime < 8):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# currentRightEETransform = robot.sensedRightEETransform()
# targetRightEETransform = (so3.mul(so3.from_moment([0,math.pi/2.0,0]),currentRightEETransform[0]),vectorops.add(currentRightEETransform[1],[-0.3,0,-0.2]))
# robot.setRightEEInertialTransform(targetRightEETransform,5)
# startTime = time.time()
# while (time.time()-startTime < 10):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(time.time()-startTime)
# print(robot.sensedRightLimbPosition())
# vis.kill()
# print("vis killed")
# print("shutting down")
# robot.shutdown()


# ##move forward
# currentLeftEETransform = robot.sensedLeftEETransform()
# targetLeftEETransform = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0.2,0,0]))
# robot.setLeftEEInertialTransform(targetLeftEETransform,5)
# startTime = time.time()
# while (time.time()-startTime < 10):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(time.time()-startTime)
# print('forwardConfig',robot.sensedLeftLimbPosition())
# ##rotate 90 degrees
# currentLeftEETransform = robot.sensedLeftEETransform()
# targetLeftEETransform = (so3.mul(so3.from_moment([math.pi/2.0,0,0]),currentLeftEETransform[0]),currentLeftEETransform[1])
# robot.setLeftEEInertialTransform(targetLeftEETransform,5)
# startTime = time.time()
# while (time.time()-startTime < 5):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(time.time()-startTime)
# print('rotatedConfig',robot.sensedLeftLimbPosition())

# ##shake hand up and down
# currentLeftEETransform = robot.sensedLeftEETransform()
# targetLeftEETransformUp = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0,0,0.1]))
# targetLeftEETransformDown = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0,0,-0.1]))

# robot.setLeftEEInertialTransform(targetLeftEETransformUp,2)
# startTime = time.time()
# while (time.time()-startTime < 2):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
# print('upConfig',robot.sensedLeftLimbPosition())
    
# robot.setLeftEEInertialTransform(targetLeftEETransformDown,4)
# startTime = time.time()
# while (time.time()-startTime < 4):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(time.time()-startTime)
# print('downConfig',robot.sensedLeftLimbPosition())
# vis.kill()
# print("vis killed")
# print("shutting down")
# robot.shutdown()


# robot = Motion(mode = 'Kinematic')
# robot.startup()
# print('Robot start() called')
# leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
# leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
# rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
# rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)
# forwardConfig = [-0.353679424536684, -2.5668469504059237, -0.7772500110039505, 3.3443770762889766, -1.1130922959204652, 0.09739075748121706]
# rotatedConfig = [-0.35670282705574874, -2.5724731944194756, -0.7598447478490323, 3.3048581414264664, -1.1410026546777612, 1.6178811981635732]
# upConfig = [-0.44262933789081105, -2.5868874513094635, -0.9121955044497172, 3.4725910883710718, -1.2268820110866854, 1.6153649982327345]
# downConfig = [-0.27958604012970756, -2.6317944630573606, -0.4794259718505932, 3.082770718089447, -1.0639381628492601, 1.6202528544627122]

# ##move to untucked...
# startTime = time.time()
# world = robot.getWorld()
# vis.add("world",world)
# vis.show()
# robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
# robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
# while (time.time()-startTime < 8):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# ##move forward
# startTime = time.time()
# robot.setLeftLimbPositionLinear(forwardConfig,3)
# while (time.time()-startTime < 5):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# ##rotate
# startTime = time.time()
# robot.setLeftLimbPositionLinear(rotatedConfig,2)
# while (time.time()-startTime < 2.5):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# ##up
# startTime = time.time()
# robot.setLeftLimbPositionLinear(upConfig,0.5)
# while (time.time()-startTime < 0.5):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# ##down
# startTime = time.time()
# robot.setLeftLimbPositionLinear(downConfig,1)
# while (time.time()-startTime < 1):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# #up again
# startTime = time.time()
# robot.setLeftLimbPositionLinear(upConfig,1)
# while (time.time()-startTime < 1):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])


# #up again
# startTime = time.time()
# robot.setLeftLimbPositionLinear(downConfig,1)
# while (time.time()-startTime < 5):
#     vis.lock()
#     #robot.setBaseVelocity([0.5,0.1])
#     vis.unlock()
#     time.sleep(0.02)
#     #print(robot.sensedLeftEETransform()[1])

# vis.kill()
# print("vis killed")
# print("shutting down")
# robot.shutdown()


###############Physical starts here #############

robot = Motion(mode = 'Physical')
robot.startup()
print('Robot start() called')
leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974] #motionAPI format
rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)
rightDownConfig = [-0.6860278288470667, -2.182815214196676, 2.0740626494037073, -0.7054613393596192, 1.5997309684753418, -1.4906814734088343]



#move to untucked...
startTime = time.time()
# robot.setRightLimbPositionLinear(rightUntuckedConfig,5)
robot.setRightLimbPositionLinear(rightDownConfig,5)
robot.setLeftLimbPositionLinear(leftUntuckedConfig,5)
time.sleep(5)

print('3')
time.sleep(1)
print('2')
time.sleep(1)
print('1')
time.sleep(1)
##move forward
currentLeftEETransform = robot.sensedLeftEETransform()
targetLeftEETransform = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0.2,0,0.1]))
robot.setLeftEEInertialTransform(targetLeftEETransform,2)
time.sleep(4)

##rotate 90 degrees
currentLeftEETransform = robot.sensedLeftEETransform()
targetLeftEETransform = (so3.mul(so3.from_moment([math.pi/2.0,0,0]),currentLeftEETransform[0]),currentLeftEETransform[1])
robot.setLeftEEInertialTransform(targetLeftEETransform,2.5)
time.sleep(2.5)
# print('3')
# time.sleep(1)
# print('2')
# time.sleep(1)
# print('1')
# time.sleep(1)

##shake hand up and down
currentLeftEETransform = robot.sensedLeftEETransform()
targetLeftEETransformUp = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0,0,0.1]))
targetLeftEETransformDown = (currentLeftEETransform[0],vectorops.add(currentLeftEETransform[1],[0,0,-0.06]))

robot.setLeftEEInertialTransform(targetLeftEETransformUp,1)
time.sleep(1.1)

robot.setLeftEEInertialTransform(targetLeftEETransformDown,1.6)
time.sleep(1.7)
robot.setLeftEEInertialTransform(targetLeftEETransformUp,1.6)
time.sleep(1.7)
robot.setLeftEEInertialTransform(targetLeftEETransformDown,1.6)
time.sleep(1.7)
robot.setLeftEEInertialTransform(targetLeftEETransformUp,1.6)
time.sleep(1.7)
robot.setLeftEEInertialTransform(targetLeftEETransformDown,1.6)
time.sleep(1.7)
print("shutting down")
robot.shutdown()
print("program ended")

