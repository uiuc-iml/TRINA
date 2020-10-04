import sys
import socket
import logging

from collections import OrderedDict

import yaml

from PyUniversalRobot.network import rtde

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

class Observer(object):
    def __init__(self, host, rtde_port=30004):
        self._quit = True

        self._robot_host = host
        self._rtde_port = rtde_port

        self._state = None
        self._version = None

    def start(self):
        # initialize RTDE
        conn = rtde.RTDE(self._robot_host, self._rtde_port)
        conn.connect()
        self._version = conn.get_controller_version()

        # configure outputs (URControl -> Python)
        self._fields = {
            'timestamp': 'DOUBLE',
            'target_q': 'VECTOR6D',
            'actual_q': 'VECTOR6D',
            'target_qd': 'VECTOR6D',
            'actual_qd': 'VECTOR6D',
            'target_qdd': 'VECTOR6D',
        }
        conn.send_output_setup(
            list(self._fields.keys()),
            list(self._fields.values()),
        )

        # start RTDE
        conn.send_start()

        self._quit = False
        while not self._quit:
            # receive the current state
            self._state = conn.receive()
            if self._state is None:
                logger.warn('lost RTDE connection -> stopping')
                break

            self.update(self._state)

        self._quit = True

        # stop RTDE
        conn.send_pause()
        conn.disconnect()

    def stop(self):
        self._quit = True

    def update(self, state):
        pass

    @property
    def state(self):
        return self._state

    @property
    def fields(self):
        return self._fields

    @property
    def version(self):
        return self._version

class LoggingObserver(Observer):
    def __init__(self, *args, **kwargs):
        self._count = kwargs.pop('count', None)
        self._kinematics = kwargs.pop('kinematics', None)
        self._offset = kwargs.pop('offset', [0, 0, 0])

        super(LoggingObserver, self).__init__(*args, **kwargs)

    def update(self, state):
        if self._count is not None and self._count <= 0:
            self.stop()
            return

        record = OrderedDict()
        # build default and joint fields
        for f in self.fields.keys():
            record[f] = getattr(state, f)

        if self._kinematics:
            target_J = self._kinematics.jacobian(state.target_q, self._offset)
            actual_J = self._kinematics.jacobian(state.actual_q, self._offset)
            target_H = self._kinematics.hessian(state.target_q, state.target_qd)

            # build cartesian fields
            record.update({
                'target_x': self._kinematics.forward(state.target_q).tolist(),
                'actual_x': self._kinematics.forward(state.actual_q).tolist(),
                'target_xd': target_J.dot(state.target_qd).tolist(),
                'actual_xd': actual_J.dot(state.actual_qd).tolist(),
                'target_xdd': (target_H.dot(state.target_qd) + target_J.dot(state.target_qdd)).tolist(),
            })

        yaml.dump_all([record], sys.stdout, explicit_start=True)

        if self._count is not None:
            self._count -= 1
            if self._count <= 0:
                self.stop()

if __name__ == '__main__':
    logging.basicConfig()

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot observer utility', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')
    parser.add_argument('--count', '-c', type=int, help='limit number of records to log')
    parser.add_argument('--kinematics', '-k', help='robot kinematics to use', choices=['UR3', 'UR5', 'UR10'])

    args = parser.parse_args()

    if args.kinematics:
        from PyUniversalRobot import kinematics
        kin = getattr(kinematics, args.kinematics)
    else:
        kin = None

    # to faciliate logging
    # https://stackoverflow.com/a/8661021
    yaml.add_representer(OrderedDict, lambda self, data: self.represent_mapping('tag:yaml.org,2002:map', data.items()))

    obs = LoggingObserver(args.robot, count=args.count, kinematics=kin)
    obs.start()
