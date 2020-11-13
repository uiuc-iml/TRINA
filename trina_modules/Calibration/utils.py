
def extractLimbPositions(q):
    """
    Given TRINA robot configuration, return the left and right limb positions
    """

    return q[7:13],q[15:21]


def getLeftlinkTransform(robot,q,link_num):
    """
    Given TRINA robot configurtion, return the left EE transform
    """
