from copy import copy

def getLeftLinkTransform(robot,q,link_num):
    """
    Given TRINA robot configurtion, return the left EE transform
    """

    initial_config = robot.getConfig()
    target_config = copy(initial_config)
    target_config[7:13] = q
    robot.setConfig(target_config)
    T = robot.link(link_num).getTransform()
    robot.setConfig(initial_config)

    return T

def getRightLinkTransform(robot,q,link_num):
    """
    Given TRINA robot configurtion, return the left EE transform
    """

    initial_config = robot.getConfig()
    target_config = copy(initial_config)
    target_config[15:21] = q
    robot.setConfig(target_config)
    T = robot.link(link_num).getTransform()
    robot.setConfig(initial_config)
    return T

def extractLimbPositions(q):
    """
    Given TRINA robot configuration, return the left and right limb positions
    """
    return q[7:13],q[15:21]
