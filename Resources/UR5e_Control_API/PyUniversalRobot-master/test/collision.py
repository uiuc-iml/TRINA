from time import time

from PyUniversalRobot.control import Observer
from PyUniversalRobot.model import CollisionModel

class CollisionCheckObserver(Observer):
    def __init__(self, *args, **kwargs):
        kinematics = kwargs.pop('kinematics')
        self._collider = kwargs.pop('collider')

        super(CollisionCheckObserver, self).__init__(*args, **kwargs)

        self._model = CollisionModel(kinematics=kinematics, collider=self._collider)

    def update(self, state):
        mark = time()
        self._model.pose(state.actual_q)

        collisions = self._collider.check()
        mark2 = time()
        dt = mark2 - mark

        for collision in collisions:
            print('{0[0].name:15} <-> {0[1].name:15}:   {1:-5.1f} mm    ({2:3.2f} ms)'.format(collision.objects, 1000 * collision.distance, 1000 * dt))

if __name__ == '__main__':
    import logging
    logging.basicConfig()

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot collision checking test', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')
    parser.add_argument('kinematics', help='robot kinematics to use', choices=['UR3', 'UR5', 'UR10'])

    args = parser.parse_args()

    from PyUniversalRobot import kinematics
    from PyCollision import Collider

    collider = Collider(1)
    collider.default_margin = 0.02

    obs = CollisionCheckObserver(
        args.robot,
        kinematics=getattr(kinematics, args.kinematics),
        collider=collider
    )
    obs.start()
