
class Model(object):
    def __init__(self, *args, **kwargs):
        self._kinematics = kwargs.pop('kinematics')

        self._tool = kwargs.pop('tool', None)
        self._mount = kwargs.pop('mount', None)

        self._links = []

    @property
    def kinematics(self):
        return self._kinematics

    @property
    def name(self):
        return self.kinematics.name

    @property
    def tool(self):
        return self._tool

    @property
    def mount(self):
        return self._mount

    @property
    def base_link(self):
        return self._links[0]

    @property
    def ee_link(self):
        return self._links[-1]

    def link(self, idx):
        if isinstance(idx, str):
            idx = self.link_names.index(idx)

        return self._links[idx]

    @property
    def joint_names(self):
        return ['Shoulder', 'Arm', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3']

    @property
    def link_names(self):
        return ['Base', 'Shoulder', 'Arm', 'Forearm', 'Hand1', 'Hand2', 'End-Effector']
