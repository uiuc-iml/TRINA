from math import pi
from random import uniform

from PyUniversalRobot.control import AntiCollisionController
from PyCollision import Collider

class KinematicsCheck(AntiCollisionController):
    def __init__(self, *args, **kwargs):
        self._kinematics = kwargs['kinematics']

        super(KinematicsCheck, self).__init__(*args, **kwargs)

        self._last_timestamp = None
        self._qt = None

    def update(self, state):
        q = state.actual_q

        T = self._kinematics.forward(q)
        qi = self._kinematics.inverse_nearest(T, q)

        # print('{:6.3f} {}'.format(sum(((q - qi) * 180 / pi)**2), ((q - qi) * 180 / pi).round(3)))
        print(T[:3, 3].T.round(5) * 1000)

        if not self._qt or not self._last_timestamp or state.timestamp - self._last_timestamp > 5:
            self._qt = [ uniform(-3, 3) for i in range(self._kinematics.dimension) ]
            self._last_timestamp = state.timestamp

        self.servo(q=self._qt)


if __name__ == '__main__':
    import logging
    logging.basicConfig()

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot kinematics test', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')

    args = parser.parse_args()

    from PyUniversalRobot import kinematics
    ctrl = KinematicsCheck('192.168.56.101', kinematics=kinematics.UR3, collider=Collider(1))
    ctrl.start()
