import socket
import logging

from PyUniversalRobot.network import rtde

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

SETPOINT_HALT     = 0
SETPOINT_POSITION = 1
SETPOINT_VELOCITY = 2

_CONTROLLER_PROGRAM = '''
stop program
set unlock protective stop

def rtde_control_loop():

    # constants
    SETPOINT_TIMEOUT  = 5
    SETPOINT_HALT     = 0
    SETPOINT_POSITION = 1
    SETPOINT_VELOCITY = 2
    CONTROL_PERIOD = 0.008
    RTDE_WATCHDOG_FREQUENCY = 1

    # integer registers
    REG_SETPOINT = 0
    REG_TYPE = 1

    # double registers
    REG_TARGET = 0
    REG_VELOCITY = 6
    REG_ACCELERATION = 7
    REG_LOOKAHEAD = 8
    REG_GAIN = 9

    # I/O configuration
    #set_tool_communication(True,115200,1,2,1.0,3.5)
    set_standard_analog_input_domain(0, 1)
    set_standard_analog_input_domain(1, 1)
    #set_tool_analog_input_domain(0, 1)
    #set_tool_analog_input_domain(1, 1)
    set_analog_outputdomain(0, 0)
    set_analog_outputdomain(1, 0)
    set_tool_voltage(24)
    set_input_actions_to_default()

    # tool configuration
    set_tcp(p[{tcp[0]}, {tcp[1]}, {tcp[2]}, {tcp[3]}, {tcp[4]}, {tcp[5]}])
    set_payload({payload})
    set_gravity([{gravity[0]}, {gravity[1]}, {gravity[2]}])

    setpoint_number = read_input_integer_register(REG_SETPOINT)
    last_setpoint_number = setpoint_number
    missed_setpoints = 0

    rtde_set_watchdog("input_int_register_0", RTDE_WATCHDOG_FREQUENCY, "stop")




    while True:
	###################
    	#rq_reset('1')
    	rq_activate(1)
    	#rq_close('1')
    	rq_open(1)
    	###################

        setpoint_number = read_input_integer_register(REG_SETPOINT)
        if setpoint_number == last_setpoint_number:
            missed_setpoints = missed_setpoints + 1
        else:
            missed_setpoints = 0
        end
        last_setpoint_number = setpoint_number

        if missed_setpoints >= SETPOINT_TIMEOUT:
            popup("setpoint timeout", title="PyUniversalRobot", error=True)
            halt
        end

        # update the setpoint
        write_output_integer_register(0, setpoint_number)
	
	###############
	# update the gripper location
	#gripper_q=rq_current_pos()
 	write_output_integer_register(7, 2)
	###############


        target = [0, 0, 0, 0, 0, 0]
        target[0] = read_input_float_register(REG_TARGET + 0)
        target[1] = read_input_float_register(REG_TARGET + 1)
        target[2] = read_input_float_register(REG_TARGET + 2)
        target[3] = read_input_float_register(REG_TARGET + 3)
        target[4] = read_input_float_register(REG_TARGET + 4)
        target[5] = read_input_float_register(REG_TARGET + 5)

        type = read_input_integer_register(REG_TYPE)
        if type == SETPOINT_HALT:
            # issue command
            halt
        elif type == SETPOINT_POSITION:
            # read lookahead and gain parameters
            lookahead = read_input_float_register(REG_LOOKAHEAD)
            gain = read_input_float_register(REG_GAIN)

            # issue command
            # NOTE: acceleration and velocity arguments are ignored
            servoj(target, 0, 0, CONTROL_PERIOD, lookahead, gain)
        elif type == SETPOINT_VELOCITY:
            # read acceleration parameter
            acceleration = read_input_float_register(REG_ACCELERATION)

            # issue command
            speedj(target, acceleration, CONTROL_PERIOD)
        else:
            # alert and quit
            popup("unknown setpoint type received", title="PyUniversalRobot", error=True)
            halt
        end
    end
end
'''

class Controller(object):
    def __init__(self, host, **kwargs):
        self._quit = True

        self._robot_host = host
        self._rtde_port = kwargs.pop('rtde_port', 30004)
        self._command_port = kwargs.pop('command_port', 30002)

        self._servo_halt = None
        self._servo_position = None
        self._servo_velocity = None

        self._gain = 300
        self._lookahead = 0.1
        self._acceleration = 10
        self._velocity = 0
        self._speed_scale = None

        self._tcp = kwargs.pop('tcp', [0,0,0.04,0,0,0])
        self._payload = kwargs.pop('payload', 0.9)
        self._gravity = kwargs.pop('gravity', [0, 0, 9.81])

        self._version = None

        self._filters = kwargs.pop('filters', [])

    def start(self):
        # initialize RTDE
        conn = rtde.RTDE(self._robot_host, self._rtde_port)
        conn.connect()
        self._version = conn.get_controller_version()

        # configure outputs (URControl -> Python)
        conn.send_output_setup(
            ['timestamp', 'target_q', 'actual_q', 'target_qd', 'actual_qd', 'target_qdd', 'target_speed_fraction','output_int_register_7'],
            ['DOUBLE', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'DOUBLE','INT32']
        )

        # configure inputs (Python -> URControl)
        target = conn.send_input_setup(['input_double_register_{}'.format(i) for i in range(6)], ['DOUBLE']*6)
        setpoint_id = conn.send_input_setup(['input_int_register_0'], ['INT32'])
        target_type = conn.send_input_setup(['input_int_register_1'], ['INT32'])
        velocity = conn.send_input_setup(['input_double_register_6'], ['DOUBLE'])
        acceleration = conn.send_input_setup(['input_double_register_7'], ['DOUBLE'])
        lookahead = conn.send_input_setup(['input_double_register_8'], ['DOUBLE'])
        gain = conn.send_input_setup(['input_double_register_9'], ['DOUBLE'])
        speed_slider = conn.send_input_setup(['speed_slider_mask', 'speed_slider_fraction'], ['UINT32', 'DOUBLE'])
	
        # start RTDE
        conn.send_start()

        # start the controller program
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((self._robot_host, self._command_port))
        program = _CONTROLLER_PROGRAM.format(tcp=self._tcp, payload=self._payload, gravity=self._gravity)
        logger.info('controller program:\n{}'.format(program))
        sock.sendall(program.encode('ascii') + b'\n')

        def r2l(reg, base=0):
            list = []
            for i in range(6):
                list.append(reg.__dict__['input_double_register_{}'.format(base + i)])
            return list

        def l2r(list, reg, base=0):
            for i in range (6):
                reg.__dict__['input_double_register_{}'.format(base + i)] = list[i]
            return reg

        max_speed_scale = None
        setpoint_number = 0
        self._quit = False
        while not self._quit:
            # receive the current state
            state = conn.receive()
            if state is None:
                logger.warn('lost RTDE connection -> stopping')
                break

            # honor speed scaling set when program started
            if max_speed_scale is None:
                max_speed_scale = state.target_speed_fraction
                self._speed_scale = state.target_speed_fraction

            # invoke update
            self.update(state)

            # run installed filters
            for f in self._filters:
                f(state)

            # determine target type
            if self._servo_halt:
                target_type.input_int_register_1 = SETPOINT_HALT
                conn.send(target_type)
                self._servo_halt = None
            elif self._servo_position:
                target_type.input_int_register_1 = SETPOINT_POSITION
                conn.send(target_type)
                conn.send(l2r(self._servo_position, target, 0))
                self._servo_position = None
            elif self._servo_velocity:
                target_type.input_int_register_1 = SETPOINT_VELOCITY
                conn.send(target_type)
                conn.send(l2r(self._servo_velocity, target, 0))
                self._servo_velocity = None
            else:
                logger.warn('missing setpoint -> stopping')
                break

            # kick watchdog
            setpoint_number += 1
            setpoint_id.input_int_register_0 = setpoint_number
            conn.send(setpoint_id)

            # clamp speed scale
            self._speed_scale = max(min(self._speed_scale, max_speed_scale), 0)

            # send parameters
            velocity.input_double_register_6 = self._velocity
            conn.send(velocity)
            acceleration.input_double_register_7 = self._acceleration
            conn.send(acceleration)
            lookahead.input_double_register_8 = self._lookahead
            conn.send(lookahead)
            gain.input_double_register_9 = self._gain
            conn.send(gain)
            speed_slider.speed_slider_mask = 1
            speed_slider.speed_slider_fraction = self._speed_scale
            conn.send(speed_slider)

        self._quit = True
        sock.sendall(b'stop program\n')

        conn.send_pause()
        conn.disconnect()

    def servo(self, halt=None, q=None, qd=None, qd_max=None, qdd_max=None, lookahead=None, gain=None):
        if sum([ x is not None for x in [halt, q, qd]]) > 1:
            raise RuntimeError('cannot specifiy more than one of halt, q, and qd')
        elif halt:
            self._servo_halt = True
            self._servo_position = None
            self._servo_velocity = None
        elif q:
            self._servo_halt = None
            self._servo_position = q
            self._servo_velocity = None
        elif qd:
            self._servo_halt = None
            self._servo_position = None
            self._servo_velocity = qd
        else:
            raise RuntimeError('missing halt, q, or qd')

        if qd_max is not None:
            self._velocity = qd_max
        if qdd_max is not None:
            self._acceleration = qdd_max
        if lookahead is not None:
            self._lookahead = lookahead
        if gain is not None:
            self._gain = gain

    def speed_scale(self, s=None):
        if s is not None:
            self._speed_scale = s

        return self._speed_scale

    def stop(self):
        self._quit = True

    def update(self, state):
        pass

    @property
    def version(self):
        return self._version

class WiggleController(Controller):
    def __init__(self, *args, **kwargs):
        self._frequency = kwargs.pop('frequency', 0.1)
        self._amplitude = kwargs.pop('amplitude', 5)

        super(WiggleController, self).__init__(*args, **kwargs)

        self._start_time = None
        self._last_t = 0

    def update(self, state):
        wait_time = 1
        from math import pi, cos

        if self._start_time is None:
            self._start_time = state.timestamp

        t = state.timestamp - self._start_time

        if t < wait_time:
            new_target_speed = [0]*6
        else:
            f = self._frequency
            A = self._amplitude/180.0*pi

            #new_target_speed = [(-2*pi*A*f)*cos(2*pi*f*(t - wait_time))]*6
	    new_target_speed = [0]*6
        self.servo(qd=new_target_speed)

        dt = t - self._last_t
        self._last_t = t
        print('{:6.4f} {:6.4f}'.format(t, dt), ' '.join(['{:6.2f}'.format(x/pi*180) for x in state.actual_q]))
	print state.output_int_register_7
class ConfigController(Controller):
    def __init__(self, *args, **kwargs):
        self._target = kwargs.pop('target')

        super(ConfigController, self).__init__(*args, **kwargs)

        self._start_time = None
        self._last_t = 0

    def update(self, state):
        from math import pi

        if self._start_time is None:
            self._start_time = state.timestamp

        t = state.timestamp - self._start_time

        self.servo(q=self._target)

        dt = t - self._last_t
        self._last_t = t
        print('{:6.4f} {:6.4f}'.format(t, dt), ' '.join(['{:6.2f}'.format(x/pi*180) for x in state.actual_q]))

if __name__ == '__main__':
    logging.basicConfig()

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot wiggle test', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')
    parser.add_argument('--frequency', '-f', type=float, help='wiggle frequency (Hz)', default=0.1)
    parser.add_argument('--amplitude', '-a', type=float, help='wiggle amplitude (deg)', default=5)

    args = parser.parse_args()

    ctrl = WiggleController(args.robot, frequency=args.frequency, amplitude=args.amplitude)
    ctrl.start()
