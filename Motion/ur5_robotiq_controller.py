#!/usr/bin/python27
#This file also contains functionalities to control the gripper. This builds upon ur5_controller.py and python-urx.

import socket
import logging
import time
#import klampt.math.vectorops
import math
from PyUniversalRobot.network import rtde
import threading
import signal
import serial
import sys
import binascii
import ur5_config as config
from copy import deepcopy

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

SETPOINT_HALT     = 0
SETPOINT_POSITION = 1
SETPOINT_VELOCITY = 2
#set_tcp(p[{tcp[0]}, {tcp[1]}, {tcp[2]}, {tcp[3]}, {tcp[4]}, {tcp[5]}])
#zero_ftsensor()
_CONTROLLER_PROGRAM = '''
stop program
set unlock protective stop

def rtde_control_loop():
    #Tear the FT sensor
    zero_ftsensor()
    # constants
    SETPOINT_TIMEOUT  = 125
    SETPOINT_HALT     = 0
    SETPOINT_POSITION = 1
    SETPOINT_VELOCITY = 2
    CONTROL_PERIOD = 0.004
    RTDE_WATCHDOG_FREQUENCY = 1

    # robotiq gripper
    GRIPPER_FLAG = {gripper_flag}

    # integer registers
    REG_SETPOINT = 0
    REG_TYPE = 1
    REG_GRIPPER = 2

    # double registers
    REG_TARGET = 0
    REG_VELOCITY = 6
    REG_ACCELERATION = 7
    REG_LOOKAHEAD = 8
    REG_GAIN = 9
    REG_G=10

    # I/O configuration
    set_standard_analog_input_domain(0, 1)
    set_standard_analog_input_domain(1, 1)
    set_tool_analog_input_domain(0, 1)
    set_tool_analog_input_domain(1, 1)
    set_analog_outputdomain(0, 0)
    set_analog_outputdomain(1, 0)
    set_input_actions_to_default()

    if GRIPPER_FLAG:
       set_tool_voltage(24)
       set_tool_communication(True,115200,0,1,1.5,3.5)
       socket_open("127.0.0.1",63352,"gripper_socket")
    else:
       set_tool_voltage(0)
    end

    # tool configuration
    set_payload_cog([{cog[0]}, {cog[1]}, {cog[2]}])
    set_payload({payload})
    set_gravity([{gravity[0]}, {gravity[1]}, {gravity[2]}])

    setpoint_number = read_input_integer_register(REG_SETPOINT)
    last_setpoint_number = setpoint_number
    missed_setpoints = 0

    rtde_set_watchdog("input_int_register_0", RTDE_WATCHDOG_FREQUENCY, "stop")

    #activate the gripper
    if GRIPPER_FLAG:
       socket_set_var("ACT", 1, "gripper_socket")  # Activate the gripper
       socket_set_var("GTO", 1, "gripper_socket")  # Enable the gripper
       sleep(1.0)
    end

    while True:
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

        target = [0, 0, 0, 0, 0, 0]
        target[0] = read_input_float_register(REG_TARGET + 0)
        target[1] = read_input_float_register(REG_TARGET + 1)
        target[2] = read_input_float_register(REG_TARGET + 2)
        target[3] = read_input_float_register(REG_TARGET + 3)
        target[4] = read_input_float_register(REG_TARGET + 4)
        target[5] = read_input_float_register(REG_TARGET + 5)

        G = [0,0,0]
        G[0] = read_input_float_register(REG_G + 0)
        G[1] = read_input_float_register(REG_G + 1)
        G[2] = read_input_float_register(REG_G + 2)
        set_gravity(G)

        if GRIPPER_FLAG:
            gripper_action = read_input_integer_register(REG_GRIPPER)
            if gripper_action == 1:
                socket_set_var("POS", 0, "gripper_socket")
            elif gripper_action == 2:
                socket_set_var("POS", 255, "gripper_socket")
            end
        end

        type = read_input_integer_register(REG_TYPE)
        if type == SETPOINT_HALT:
            # issue command
            popup("halt command issued", title="PyUniversalRobot", error=True)
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



class UR5Controller(object):
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

        self._cog = kwargs.pop('cog', [0.0,0.0,0.0])
        self._payload = kwargs.pop('payload', 0.0)
        self._gravity = kwargs.pop('gravity', [0, 0, 9.82])

        self._version = None

        self._filters = kwargs.pop('filters', [])
        self._start_time = None
        self._last_t = 0

        #stuff needed for threading
        self._conn = None
        self._max_speed_scale = None
        self._sock = None

        self._running = False
        self._no_connection = False

        self._gripper = kwargs.get('gripper', False)

        self._new_gripper_action = False
        self._gripper_action = 1 #1 close 2 open

    def start(self):
        # initialize RTDE
        self._conn = rtde.RTDE(self._robot_host, self._rtde_port)
        self._conn.connect()
        self._version = self._conn.get_controller_version()

        # configure outputs (URControl -> Python)
        self._conn.send_output_setup(
             ['timestamp', 'target_q', 'actual_q', 'target_qd', 'actual_qd', 'target_qdd', 'target_speed_fraction','actual_TCP_force'],  # Add TCP forces here
            ['DOUBLE', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'DOUBLE', 'VECTOR6D']
        )

        # configure inputs (Python -> URControl)
        target = self._conn.send_input_setup(['input_double_register_{}'.format(i) for i in range(6)], ['DOUBLE']*6)
        setpoint_id = self._conn.send_input_setup(['input_int_register_0'], ['INT32'])
        target_type = self._conn.send_input_setup(['input_int_register_1'], ['INT32'])
        velocity = self._conn.send_input_setup(['input_double_register_6'], ['DOUBLE'])
        acceleration = self._conn.send_input_setup(['input_double_register_7'], ['DOUBLE'])
        lookahead = self._conn.send_input_setup(['input_double_register_8'], ['DOUBLE'])
        gain = self._conn.send_input_setup(['input_double_register_9'], ['DOUBLE'])
        speed_slider = self._conn.send_input_setup(['speed_slider_mask', 'speed_slider_fraction'], ['UINT32', 'DOUBLE'])
        gravity=self._conn.send_input_setup(['input_double_register_{}'.format(i+10) for i in range(3)], ['DOUBLE']*3)
        #set up gripper here

        if self._gripper:
            gripper_action = self._conn.send_input_setup(['input_int_register_2'], ['INT32'])

        # start RTDE
        self._conn.send_start()

        # start the controller program
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._robot_host, self._command_port))
        program = _CONTROLLER_PROGRAM.format(cog=self._cog, payload=self._payload, gravity=self._gravity, gripper_flag = self._gripper)
        logger.info('controller program:\n{}'.format(program))
        self._sock.sendall(program.encode('ascii') + b'\n')

        self._max_speed_scale = None
        if self._gripper:
            controlThread = threading.Thread(target = self.controlLoop, args=[[target, setpoint_id, target_type, velocity, acceleration, lookahead, gain, speed_slider,gravity,\
                gripper_action]])
        else:
            controlThread = threading.Thread(target = self.controlLoop, args=[[target, setpoint_id, target_type, velocity, acceleration, lookahead, gain, speed_slider,gravity]])

        controlThread.start()
        #return started
        return True

    def controlLoop(self, args):
        if self._gripper:
            if not len(args) == 10:
                print("Error in thread configuration, should have 10 inputs")
                self._sock.sendall(b'stop program\n')
                self._conn.send_pause()
                self._conn.disconnect()
                #disconnect and exit
                return
        else:
            if not len(args) == 9:
                print("Error in thread configuration, should have 9 inputs")
                self._sock.sendall(b'stop program\n')
                self._conn.send_pause()
                self._conn.disconnect()
                #disconnect and exit
                return

        target = args[0]
        setpoint_id = args[1]
        target_type = args[2]
        velocity = args[3]
        acceleration = args[4]
        lookahead = args[5]
        gain = args[6]
        speed_slider = args[7]
        gravity = args[8]

        if self._gripper:
            gripper_action = args[9]


        def r2l(reg, base=0):
            list = []
            for i in range(6):
                list.append(reg.__dict__['input_double_register_{}'.format(base + i)])
            return list

        def l2r(list, reg, base=0):
            for i in range (6):
                reg.__dict__['input_double_register_{}'.format(base + i)] = list[i]
            return reg

        def l2r2(list,reg,base=0):
            for i in range (3):
                reg.__dict__['input_double_register_{}'.format(base + i)] = list[i]
            return reg

        self._quit = False

        setpoint_number = 0
        self._running = True
        #could set this outside the thread, but it seems reasonable for when the thread starts the setpoint should be 1

        while not self._quit:
            # receive the current update_state
            # start_time = time.time()

            state = self._conn.receive()
            if state is None:
                logger.warn('lost RTDE connection -> stopping')
                self._running = False
                self._no_connection = True
                break
            # honor speed scaling set when program started
            if self._max_speed_scale is None:
                self._max_speed_scale = state.target_speed_fraction
                self._speed_scale = state.target_speed_fraction

            # invoke update
            self._update(state)

            # run installed filters
            for f in self._filters:
                f(state)

            # determine target type
            if self._servo_halt:
                target_type.input_int_register_1 = SETPOINT_HALT
                self._conn.send(target_type)
                self._servo_halt = None
            elif self._servo_position:
                target_type.input_int_register_1 = SETPOINT_POSITION
                self._conn.send(target_type)
                self._conn.send(l2r(self._servo_position, target, 0))
                self._servo_position = None
            elif self._servo_velocity:
                target_type.input_int_register_1 = SETPOINT_VELOCITY
                self._conn.send(target_type)
                self._conn.send(l2r(self._servo_velocity, target, 0))
                self._servo_velocity = None
            else:
                logger.warn('missing setpoint -> stopping')
                break

            # kick watchdog
            setpoint_number += 1
            setpoint_id.input_int_register_0 = setpoint_number
            self._conn.send(setpoint_id)

            # clamp speed scale
            self._speed_scale = max(min(self._speed_scale, self._max_speed_scale), 0)

            # send parameters
            velocity.input_double_register_6 = self._velocity
            self._conn.send(velocity)
            acceleration.input_double_register_7 = self._acceleration
            self._conn.send(acceleration)
            lookahead.input_double_register_8 = self._lookahead
            self._conn.send(lookahead)
            gain.input_double_register_9 = self._gain
            self._conn.send(gain)
            speed_slider.speed_slider_mask = 1
            speed_slider.speed_slider_fraction = self._speed_scale
            self._conn.send(speed_slider)

            # send gravity
            self._conn.send(l2r2(self._gravity,gravity,10))

            #send gripper action
            if self._gripper:
                self._new_gripper_action = True
                if self._new_gripper_action:
                    gripper_action.input_int_register_2 = self._gripper_action
                    self._new_gripper_action = False
                else:
                    gripper_action.input_int_register_2 = 0
                self._conn.send(gripper_action)
                
                # print('gripper action:',gripper_action)

            # elapsed_time = time.time() - start_time
            # freq = 1.0/elapsed_time
            # if freq < 100:
            #     print('arm controller Hz:',freq)

        self._quit = True
        print("ending control loop")
        #if this loop exits, disconnect
        self._sock.sendall(b'stop program\n')
        self._conn.send_pause()
        self._conn.disconnect()
        print("disconnecting")


    def servo(self, halt=None, q=None, qd=None, qd_max=None, qdd_max=None, lookahead=None, gain=None,g=None):
        if sum([ x is not None for x in [halt, q, qd]]) > 1:
            raise RuntimeError('cannot specifiy more than one of halt, q, and qd')
        elif halt:
            self._servo_halt = True
            self._servo_position = None
            self._servo_velocity = None
        elif q:
            self._servo_halt = None
            self._servo_position = deepcopy(q)
            self._servo_velocity = None
        elif qd:
            self._servo_halt = None
            self._servo_position = None
            self._servo_velocity = deepcopy(qd)
        else:
            raise RuntimeError('missing halt, q, or qd')
        if g is not None:
            self._gravity = deepcopy(g)
        if qd_max is not None:
            self._velocity = qd_max
        if qdd_max is not None:
            self._acceleration = qdd_max
        if lookahead is not None:
            self._lookahead = lookahead
        if gain is not None:
            self._gain = gain

    #this function is not used
    #speed_scale set on teaching pendant directly
    def speed_scale(self, s=None):
        if s is not None:
            self._speed_scale = s

        return self._speed_scale

    def _update(self, state):
    	pass

    def stop(self):
        # if self._gripper:
        #     self._new_gripper_action = True
        #     self._gripper_action = 2
        #     time.sleep(0.05)
        self._quit = True

    def inLimits(self, q, min_limits=None, max_limits=None):
        for i in range(0, len(q)):
            if(min_limits and max_limits):
                if not(q[i] <= max_limits[i] and q[i] >= min_limits[i]):
                    return False
            else:
                print("warning, joint limits not set")
                return False
        return True

    def isFormatted(self, val):
        #do formatting
        if val:
            if len(val) == config.ROBOT_CONFIG_LEN:
                return True
        else:
            print("Error, val: ", val, " is not formatted correctly")
        return False

    def running(self):
        return self._running

    def connected(self):
        return (not self._no_connection)

    def setGravity(self,g):
            self._gravity = deepcopy(g)

    def openGripper(self):
        self._new_gripper_action = True
        self._gripper_action = 2

    def closeGripper(self):
        self._new_gripper_action = True
        self._gripper_action = 1

    @property
    def version(self):
        return self._version
