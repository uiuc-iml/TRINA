from copy import copy

def col_to_row_major(R):
    return [R[0],R[3],R[6],R[1],R[4],R[7],R[2],R[3],R[6]]


def getLeftLinkTransform(robot,q,link_num,model):
    """
    Given TRINA robot configurtion, return the left EE transform
    """

    initial_config = robot.getConfig()
    target_config = copy(initial_config)
    if model == 'bubonic':
        target_config[7:13] = q
    elif model == 'cholera':
        target_config[11:17] = q
    robot.setConfig(target_config)
    T = robot.link(link_num).getTransform()
    robot.setConfig(initial_config)

    return T

def getRightLinkTransform(robot,q,link_num,model):
    """
    Given TRINA robot configurtion, return the left EE transform
    """

    initial_config = robot.getConfig()
    target_config = copy(initial_config)
    if model == 'bubonic':
        target_config[15:21] = q
    elif model == 'cholera':
        target_config[19:25] = q
    robot.setConfig(target_config)
    T = robot.link(link_num).getTransform()
    robot.setConfig(initial_config)
    return T

def extractLimbPositions(q,model):
    """
    Given TRINA robot configuration, return the left and right limb positions
    """
    if model == 'bubonic':
        return q[7:13],q[15:21]
    elif model == 'cholera':
        return q[11:17],q[19:25]