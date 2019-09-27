import logging

import time
import numpy as np
from klampt import *
from klampt.math import vectorops,so3,se3
from klampt.io import loader
from klampt.model import ik
import math
import random
from robot_api.RobotController import UR5WithGripperController
from scipy import signal
from klampt import vis
from klampt.model import collide
from copy import deepcopy

#MODE = "Debugging"
MODE = "Physical"

gripperDistance = 0.15
IKDeviation = 5
gridSize = 0.06
tuckingTime = 7
def left_2_right(config):
    RConfig = []
    RConfig.append(-config[0])
    RConfig.append(-config[1]-math.pi)
    RConfig.append(-config[2])
    RConfig.append(-config[3]+math.pi)
    RConfig.append(-config[4])
    RConfig.append(-config[5])
    RConfig.append(0)

    for ele in RConfig:
        if ele >= 2*math.pi or ele <= -2*math.pi:
            print 'out of range..'
            return []
    return RConfig

leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172, 0]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974,0]
waypoint1 = [0.7934980392456055, -2.541288038293356+math.pi/2, -2.7311811447143555, 4.664876623744629, -0.04916698137392217, 0.09736919403076172, 0]

rightTuckedConfig = left_2_right(leftTuckedConfig)
rightUntuckedConfig = left_2_right(leftUntuckedConfig)
waypoint2 = left_2_right(waypoint1)



def constantVServo(controller,servoTime,target,dt):
    currentTime=0.0
    goalConfig=deepcopy(target)
    currentConfig=controller.getConfig()
    #print goalConfig
    difference=vectorops.sub(goalConfig,currentConfig)

    while currentTime < servoTime:
        setConfig=vectorops.madd(currentConfig,difference,currentTime/servoTime)
        controller.setConfig(setConfig)
        time.sleep(dt)
        currentTime=currentTime+dt
        #print currentTime

def controller_2_klampt(robot,controllerQ):
    qOrig=robot.getConfig()
    q=[v for v in qOrig]
    for i in range(6):
        q[i+1]=controllerQ[i]
    return q

def klampt_2_controller(robotQ):

    temp=robotQ[1:7]
    temp.append(0)
    return temp

def set_Partial_Config(robot,trans,rot,leftArm,rightArm): #takes in controller format
    baseQ = [0,trans,rot,0,0,0,0]
    leftArmQ = [0] + leftArm[0:6] + [0]
    rightArmQ = [0]+ rightArm[0:6] + [0]
    leftGripperQ= [0,0,0,0,0,0,0,0,0]
    rightGripperQ = [0,0,0,0,0,0,0,0,0]
    q = baseQ + leftArmQ + leftGripperQ +rightArmQ + rightGripperQ
    robot.setConfig(q)

def get_Partial_Config(robot,trans,rot,leftArm,rightArm):
    baseQ = [0,trans,rot,0,0,0,0]
    leftArmQ = [0] + leftArm[0:6] + [0]
    leftGripperQ= [0,0,0,0,0,0,0,0,0]
    #rightArmQ = [0,-2.34,4.84,-1.4,-0.12,-0.36,0,0]
    rightArmQ = [0]+ rightArm[0:6] + [0]
    rightGripperQ = [0,0,0,0,0,0,0,0,0]
    q = baseQ + leftArmQ + leftGripperQ +rightArmQ + rightGripperQ
    return q


def gripper_transform(robot,link):
    pose = link.getTransform()
    R = [1,0,0,0,1,0,0,0,1]
    return se3.mul(pose,(R,[gripperDistance,0,0]))

def grid_EE_Position(p,xRange,yRange,zRange):
    positions = []
    xList = []
    yList = []
    zList = []
    current = xRange[0]
    while current < xRange[1]:
        xList.append(p[0]+current)
        current = current + gridSize
    
    current = yRange[0]
    while current < yRange[1]:
        yList.append(p[1]+current)
        current = current + gridSize
    
    current = zRange[0]
    while current < zRange[1]:
        zList.append(p[2]+current)
        current = current + gridSize

    for x in xList:
        for y in yList:
            for z in zList:
                positions.append([x,y,z])
    return positions

def get_Arm_Config(robot):
    q = robot.getConfig()
    return q[8:14]


def solve_IK(robot,link,positions,activeDoFs,collider,vis):
    configurations = []
    counter = 0
    initialConfig = robot.getConfig()
    for ele in positions:
        tmp = gripper_transform(robot,link)
        local1=[gripperDistance,0,0]
        local2=vectorops.add(local1,[1,0,0])
        local3=vectorops.add(local1,[0,1,0])
        pt1 = ele
        pt2 = vectorops.add(so3.apply(tmp[0],[1,0,0]),pt1)
        pt3 = vectorops.add(so3.apply(tmp[0],[0,1,0]),pt1)


        goal = ik.objective(link,local=[local1,local2,local3],world=[pt1,pt2,pt3])
        if ik.solve_nearby(goal,maxDeviation=IKDeviation,tol=0.00001,activeDofs=activeDoFs):
        #if ik.solve_global(goal,tol=0.00001,activeDofs=activeDoFs):
            if get_Arm_Config(robot)[4] < 0:
                print get_Arm_Config(robot)[4]
            collisions = collider.robotSelfCollisions(robot) 
            colCounter = 0
            for col in collisions:
                colCounter = colCounter + 1
            if colCounter > 0:
                vis.add(str(counter),ele)
                vis.setColor(str(counter),1,0,0,1)
                vis.hideLabel(str(counter),hidden=True)
                #for col in collisions:
                #   print collider.getGeomIndex(col[0]),collider.getGeomIndex(col[1])
                #add_ghosts(robot,vis,[robot.getConfig()],counter)
            else:
                configurations.append(robot.getConfig())
                vis.add(str(counter),ele)
                vis.setColor(str(counter),0,1,0,1)
                vis.hideLabel(str(counter),hidden=True)
                #add_ghosts(robot,vis,[robot.getConfig()],counter)

        else:
            pass
            #print ('IK solve failure')
            vis.add(str(counter),ele)
            vis.setColor(str(counter),0,0,1,1)
            vis.hideLabel(str(counter),hidden=True)



        counter = counter + 1
        robot.setConfig(initialConfig)  
    return configurations

def check_collision_linear(robot,q1,q2,disrectization):
    lin = np.linspace(0,1,disrectization)
    initialConfig = robot.getConfig()
    diff = vectorops.sub(q2,q1)
    counter = 0
    for c in lin:
        Q=vectorops.madd(q1,diff,c)
        robot.setConfig(Q)
        collisions = collider.robotSelfCollisions(robot) 
        colCounter = 0
        for col in collisions:
            colCounter = colCounter + 1
        if colCounter > 0:
            return True
        #add_ghosts(robot,vis,[robot.getConfig()],counter)
        counter = counter + 1
    robot.setConfig(initialConfig)
    return False


def add_ghosts(robot,vis,Qs,counter):
    #counter = 0
    #initialConfig = robot.getConfig()
    for ele in Qs:
        vis.add("ghost"+str(counter),ele)
        vis.setColor("ghost"+str(counter),0,1,0,0.5)    
        counter = counter + 1
    #robot.setConfig(initialConfig)

def add_ghosts_linear(robot,vis,start,end,discretization):
    Qs = []
    lin = np.linspace(0,1,discretization)
    diff = vectorops.sub(end,start)
    for c in lin:
        Q=vectorops.madd(start,diff,c)
        Qs.append(Q)
    add_ghosts(robot,vis,Qs,0)

def get_configs_linear(start,end,discretization):
    Qs = []
    lin = np.linspace(0,1,discretization)
    diff = vectorops.sub(end,start)
    for c in lin:
        Qs.append(vectorops.madd(start,diff,c))
    return Qs

def degree_2_radian(degree):
    radian = []
    for q in degree:
        radian.append(math.pi*float(q)/180.0)
    return radian

def untuck_arm_debug(robot,vis):

    leftTuckedConfig = [0.7919783592224121, -2.7, -2.808842658996582, 4.664960547084473, -0.039417425738733, 0.09733343124389648, 0]
    leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974,0]
    waypoint1 = [0.7919783592224121, -2.7+math.pi/2, -2.808842658996582, 4.664960547084473, -0.039417425738733, 0.09733343124389648, 0]
    
    fullLeftTucked = get_Partial_Config(robot,0.2,0,leftTuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullWaypoint1 = get_Partial_Config(robot,0.2,0,waypoint1,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullLeftUntucked = get_Partial_Config(robot,0.2,0,leftUntuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    #add_ghosts_linear(robot,vis,fullLeftTucked,fullWaypoint1,5)
    if check_collision_linear(robot,get_Partial_Config(robot,0.2,0,leftTuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0]),\
        get_Partial_Config(robot,0.2,0,waypoint1,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0]),20):
        print "collision"
    else:
        print "No collision"

    #add_ghosts_linear(robot,vis,fullWaypoint1,fullLeftUntucked,3)

    if check_collision_linear(robot,get_Partial_Config(robot,0.2,0,leftUntuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0]),\
        get_Partial_Config(robot,0.2,0,waypoint1,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0]),20):
        print "collision"
    else:
        print "No collision"


    #Display a video
    Qs = []
    Qs = Qs + get_configs_linear(fullLeftTucked,fullWaypoint1,30)
    Qs = Qs + get_configs_linear(fullWaypoint1,fullLeftUntucked,30)
    counter = 0
    vis.show()
    while vis.shown():
        if counter > 59:
            counter = 0
        vis.lock()
        robot.setConfig(Qs[counter])
        vis.unlock()
        counter = counter + 1
        time.sleep(0.1)
    return 0

def untuck_arm(robot,leftControlApi,rightControlApi,left=False,right=False):
    fullLeftTucked = get_Partial_Config(robot,0.2,0,leftTuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullWaypoint1 = get_Partial_Config(robot,0.2,0,waypoint1,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullLeftUntucked = get_Partial_Config(robot,0.2,0,leftUntuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullRightTucked = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],left_2_right(leftTuckedConfig))
    fullWaypoint2 = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],left_2_right(waypoint1))
    fullRightUntucked = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],left_2_right(leftUntuckedConfig))

    if left and right:
        print "both arms are set to be true.. exiting"
        return 0
    if (not right) and (not left):
        print "both arms are set to be false.. exiting"
        return 0

    if left:
        controlApi = leftControlApi
        tuckedConfig = leftTuckedConfig
        waypoint = waypoint1
        untuckedConfig = leftUntuckedConfig
        fullTucked = fullLeftTucked
        fullWaypoint =fullWaypoint1
        fullUntucked =fullLeftUntucked
        print ("untucking left arm")
    elif right:
        controlApi = rightControlApi
        tuckedConfig = rightTuckedConfig
        waypoint = waypoint2
        untuckedConfig = rightUntuckedConfig
        fullTucked = fullRightTucked
        fullWaypoint =fullWaypoint2
        fullUntucked =fullRightUntucked
        print ("untucking right arm")
    
    currentConfig = controlApi.getConfig()
    if left:  
        fullCurrent = get_Partial_Config(robot,0.2,0,currentConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    elif right:
        fullCurrent = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],currentConfig)
    
    if vectorops.norm(vectorops.sub(currentConfig,untuckedConfig)) < 0.03:
        print "already untucked left arm.."
        return 0    

    if vectorops.norm(vectorops.sub(currentConfig,tuckedConfig)) > 0.03:
        print 'left arm not in tucked position.. replanning tucking..'
        if not check_collision_linear(robot,fullCurrent,fullWaypoint,20):
            constantVServo(controlApi,tuckingTime,waypoint,0.004)
            constantVServo(controlApi,tuckingTime,untuckedConfig,0.004)
        else:
            print 'replanning failed..'
    else:
        if not check_collision_linear(robot,fullCurrent,fullWaypoint,20):
            constantVServo(controlApi,tuckingTime,waypoint,0.004)
            constantVServo(controlApi,tuckingTime,untuckedConfig,0.004)
        else:
            print 'left arm collision...'

    return 0


def tuck_arm(robot,leftControlApi,rightControlApi,left=False,right=False):
    fullLeftTucked = get_Partial_Config(robot,0.2,0,leftTuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullWaypoint1 = get_Partial_Config(robot,0.2,0,waypoint1,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullLeftUntucked = get_Partial_Config(robot,0.2,0,leftUntuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    fullRightTucked = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],left_2_right(leftTuckedConfig))
    fullWaypoint2 = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],left_2_right(waypoint1))
    fullRightUntucked = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],left_2_right(leftUntuckedConfig))
   
    if left and right:
        print "both arms are set to be true.. exiting"
        return 0
    if (not right) and (not left):
        print "both arms are set to be false.. exiting"
        return 0

    if left:
        controlApi = leftControlApi
        tuckedConfig = leftTuckedConfig
        waypoint = waypoint1
        untuckedConfig = leftUntuckedConfig
        fullTucked = fullLeftTucked
        fullWaypoint =fullWaypoint1
        fullUntucked =fullLeftUntucked
        print ("untucking left arm")

    elif right:
        controlApi = rightControlApi
        tuckedConfig = rightTuckedConfig
        waypoint = waypoint2
        untuckedConfig = rightUntuckedConfig
        fullTucked = fullRightTucked
        fullWaypoint =fullWaypoint2
        fullUntucked =fullRightUntucked
        print ("untucking right arm")
    
    currentConfig = controlApi.getConfig()
    if left:  
        fullCurrent = get_Partial_Config(robot,0.2,0,currentConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    elif right:
        fullCurrent = get_Partial_Config(robot,0.2,0,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0],currentConfig)


    if vectorops.norm(vectorops.sub(currentConfig,tuckedConfig)) < 0.03:
        print "already tucked"
        return 0
    if not check_collision_linear(robot,fullCurrent,fullWaypoint,20):
        constantVServo(controlApi,tuckingTime,waypoint,0.004)
        constantVServo(controlApi,tuckingTime,tuckedConfig,0.004)
    else:
        print "collisions"

    return 0

world = WorldModel()
res=world.readFile('TRINA_world.xml')
if not res:
    raise RuntimeError("unable to load model")
robot = world.robot(0)

vis.add("world",world)
collider = collide.WorldCollider(world)
leftEE = 14
leftEELink = robot.link(leftEE)
translationQ = 0.2
activeDoFs = [8,9,10,11,12,13]

zeroConfiguration = [0,-math.pi/2.0,0,-math.pi/2.0,0,0,0] #controller format


print '---------------------model loaded -----------------------------' 

if MODE == "Physical":
    
    # control interface
    
    rightArmControlApi= UR5WithGripperController(host='192.168.0.177',gripper=False,gravity=[4.91,-4.91,-6.9438])
    rightArmControlApi.start()
    time.sleep(0.5)
    #print rightArmControlApi.getConfig()#
    print '---------------------right arm started -----------------------------'

    
    

    leftArmControlApi= UR5WithGripperController(host='192.168.0.207',gripper=False,gravity=[-4.91,-4.91,-6.9366509999999995])
    leftArmControlApi.start()
    
    print '---------------------left arm started -----------------------------'
    time.sleep(0.5)
    #print leftArmControlApi.getConfig()
    untuck_arm(robot,leftArmControlApi,[],left=True,right=False)
    untuck_arm(robot,[],rightArmControlApi,left=False,right=True)
    tuck_arm(robot,leftArmControlApi,[],left=True,right=False)
    tuck_arm(robot,[],rightArmControlApi,left=False,right=True)

    rightArmControlApi.stop()
    leftArmControlApi.stop()
    
elif MODE == "Debugging":

    localGravity = [4.91,4.91,-6.9438]
    leftShoulderLink = robot.link(7)
    #set_Partial_Config(robot,translationQ,0,leftTuckedConfig,left_2_right(leftTuckedConfig))
    set_Partial_Config(robot,translationQ,0,leftUntuckedConfig,left_2_right(leftUntuckedConfig))
    #add_ghosts_linear(robot,vis,start,end,discretization)
    #untuck_arm_debug(robot,vis)
    #fullLeftTucked = get_Partial_Config(robot,0.2,0,leftTuckedConfig,[0,-math.pi/2.0,0,-math.pi/2.0,0,0,0])
    #print check_collision_linear(robot,fullLeftTucked,fullLeftTucked,1)
    vis.show()
    while vis.shown():
       time.sleep(1.0)
    #check_collision_linear(robot,q1,q2,disrectization):

'''

##########################################################
## Record some home configuration
leftArmHomeConfig=[2.8407769203186035, -1.0762546819499512, 1.1178444067584437, -1.8262273273863734, -0.8546517531024378, 1.92586088180542, 0]
#controller format
leftArmControlApi.setConfig(leftArmHomeConfig)
time.sleep(3)
print '---------------------left arm in home configuration -----------------------------'

rightArmHomeConfig=[3.337040424346924, -2.2251135311522425, -1.214920997619629, -0.8254898351481934, 0.8483424186706543, 1.568425178527832, 0]
rightArmControlApi.setConfig(rightArmHomeConfig)
time.sleep(3)
print '---------------------right arm in home configuration -----------------------------'





totalTime=12
loopTime=0.0
dt=0.005
while loopTime < totalTime:
    addition=0.1*math.sin(loopTime)
    additionVector=[addition,0,addition,addition,0,0,0]

    desiredLeftConfig=vectorops.add(leftArmHomeConfig,additionVector)
    leftArmControlApi.setConfig(desiredLeftConfig)
    desiredRightConfig=vectorops.add(rightArmHomeConfig,additionVector)
    rightArmControlApi.setConfig(desiredRightConfig)

    time.sleep(dt)
    loopTime=loopTime+dt

#rightConfig=rightArmControlApi.getConfig()
#print rightConfig
time.sleep(0.5)
leftArmControlApi.stop()
rightArmControlApi.stop()
'''