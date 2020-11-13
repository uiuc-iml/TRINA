
def extractLimbPositions(q):
    """
    Given TRINA robot configuration, return the left and right limb positions
    """

    return q[15:21],q[35:41]


def getLeftEETransform(robot,q):
    