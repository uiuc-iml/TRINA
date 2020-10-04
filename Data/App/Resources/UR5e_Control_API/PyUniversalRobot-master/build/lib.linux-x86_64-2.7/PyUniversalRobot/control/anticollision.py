import socket
import logging

from . import Controller

from PyUniversalRobot.model import CollisionModel

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class AntiCollisionController(Controller):
    def __init__(self, *args, **kwargs):
        kinematics = kwargs['kinematics']
        self._collider = kwargs['collider']
        self._slow_margin = kwargs.pop('slow_margin', 0.02)
        self._stop_margin = kwargs.pop('stop_margin', 0.01)
        self._precollision_scale = kwargs.pop('precollision_scale', 0.1)

        if self._slow_margin < self._stop_margin:
            raise ValueError('slow_margin ({}) is less than stop_margin ({})'.format(self._slow_margin, self._stop_margin))
        if self._stop_margin < 0:
            raise ValueError('stop_margin ({}) is negative'.format(self._stop_margin))
        if self._precollision_scale > 1 or self._precollision_scale < 0:
            raise ValueError('precollision_scale ({}) is not in [0, 1]'.format(self._precollision_scale))

        # insert collision checking filter
        kwargs.setdefault('filters', []).append(self._check)

        super(AntiCollisionController, self).__init__(*args, **kwargs)

        # initialize collision model
        kwargs['margin'] = self._slow_margin
        self._model = CollisionModel(**kwargs)

        self._last_scale = None

    def _check(self, state):
        # check for collisions
        self._model.pose(state.target_q)
        collisions = self._collider.check()
        if collisions:
            # find closest collision
            limiting_collision = min(collisions, key=lambda c: c.distance)
            distance = limiting_collision.distance

            if distance > 0:
                # no problem due to margin
                self.speed_scale(1)
            elif distance > -(self._slow_margin - self._stop_margin):
                print(limiting_collision)
                if distance > -(self._slow_margin - self._stop_margin) / 2:
                    # linear slowdown for first half of slow margin
                    self.speed_scale((1 - distance / (-(self._slow_margin - self._stop_margin) / 2)) * (1 - self._precollision_scale) + self._precollision_scale)
                else:
                    # fixed slow speed for second half of slow margin
                    self.speed_scale(self._precollision_scale)
            else:
                # stuck here in collision
                # NOTE: setting the speed scale to zero suspends the robot controller program so halt instead for now
                self.speed_scale(0.01)
                self.servo(halt=True)

            print(limiting_collision, self.speed_scale())
        else:
            # no collision
            self.speed_scale(1)
