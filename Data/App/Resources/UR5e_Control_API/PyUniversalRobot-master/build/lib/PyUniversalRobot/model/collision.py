import os

from PyUniversalRobot.data import DATA_ROOT
from .model import Model

class CollisionModel(Model):
    def __init__(self, *args, **kwargs):
        super(CollisionModel, self).__init__(*args, **kwargs)

        self._collider = kwargs.pop('collider')
        margin = kwargs.pop('margin', self._collider.default_margin)

        self._links = []
        for i in range(self.kinematics.dimension + 1):
            path = os.path.join(DATA_ROOT, self.kinematics.name, 'link{}_col_cvx.obj'.format(i))
            self._links.append(self.collider.add(self.link_names[i], path, margin))

        # disable collisions between adjacent links
        for i in range(self.kinematics.dimension):
            self.collider.disable(self._links[i], self._links[i + 1])

        # disable collisions of additional close links
        self.collider.disable(self.link('Base'), self.link('Arm'))
        self.collider.disable(self.link('Forearm'), self.link('Hand2'))
        self.collider.disable(self.link('Hand1'), self.link('End-Effector'))

        if self.mount:
            # disable mount and base collisions
            self.collider.disable(self.mount, self.base_link)

        if self.tool:
            # disable end-effector and tool collisions
            self.collider.disable(self.ee_link, self.tool)

    def pose(self, q, base=None):
        if self.mount:
            if base is not None:
                raise RuntimeError('cannot set base when mount is present')

            world2base = self.mount.pose
        elif base is not None:
            world2base = base
        else:
            world2base = self.base_link.pose

        # update base collidable
        self.base_link.pose = world2base

        # update the non-base link collidables
        for (link, base2link) in zip(self._links[1:], self.kinematics.forward_all(q)):
            link.pose = world2base.dot(base2link)

        if self.tool:
            # update the tool collidable
            self.tool.pose = self.ee_link.pose

    @property
    def collider(self):
        return self._collider
