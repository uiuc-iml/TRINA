leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172, 0]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974,0]
waypoint1 = [0.7934980392456055, -2.541288038293356+math.pi/2, -2.7311811447143555, 4.664876623744629, -0.04916698137392217, 0.09736919403076172, 0]

rightTuckedConfig = left_2_right(leftTuckedConfig)
rightUntuckedConfig = left_2_right(leftUntuckedConfig)
waypoint2 = left_2_right(waypoint1)

    def tuckArm(robot,arm='left'):  ##Under construction
        """While tucking arms, the other components are not allowed to move"""
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
        return

    def untuck_arm(robot,leftControlApi,rightControlApi,left=False,right=False):##Under construction
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