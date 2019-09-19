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

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name

SETPOINT_HALT     = 0
SETPOINT_POSITION = 1
SETPOINT_VELOCITY = 2

MODES = ["discrete", "continuous"]
MODE_DISCRETE = MODES[0]
MODE_CONTINUOUS = MODES[1]

#number of elements in a configuration
ROBOT_CONFIG_LEN =7
UR5_CONFIG_LEN=6

#time for robot to pause as gripper closes
GRIPPER_DELAY = 1

_CONTROLLER_PROGRAM = '''
stop program
set unlock protective stop

def rtde_control_loop():

    # constants
    SETPOINT_TIMEOUT  = 20
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
    set_standard_analog_input_domain(0, 1)
    set_standard_analog_input_domain(1, 1)
    set_tool_analog_input_domain(0, 1)
    set_tool_analog_input_domain(1, 1)
    set_analog_outputdomain(0, 0)
    set_analog_outputdomain(1, 0)
    set_tool_voltage(0)
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

	write_output_integer_register(7, 5)

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

def isFormatted(val):
    #do formatting
    if val:
        if len(val) == ROBOT_CONFIG_LEN:
            return True
    else:
        print("Error, val: ", val, " is not formatted correctly")
    return False

def ur5_diff(q_a, q_b=[0]*ROBOT_CONFIG_LEN):
    if isFormatted(q_a) and isFormatted(q_b):
        mysum = 0
        for i in range(UR5_CONFIG_LEN):
            mysum = mysum + (q_a[i] - q_b[i])**2
            #print("diff sum is ", mysum)
        return mysum
    print("Warning, not formatted correctly")

def inJointLimits(q):
    if isFormatted(q):
        for i in range(0, len(q)):
            if not(q[i] <= math.pi*2 and q[i] >= -2*math.pi):
                return False
        return True
    return False


class Ur5Controller(object):
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
        self._start_time = None
        self._last_t = 0

        #stuff needed for threading
        self._conn = None
        self._max_speed_scale = None
        self._sock = None

        self._path = []
        self._q_curr = []
        self._qdot_curr = []
        self._q_commanded = []

        self._mode = MODE_DISCRETE

        self._gripper_delay_t = 0
        self._gripper_width = 1
        self._gripper_ser = None
        #1 is open, 0 is closed

        self._eps = 1e-7
        self._max_eps = 1e-7
        self._speed = 3
        self._max_speed = 6


        self._pathLock = threading.Lock()
        #signal.signal(signal.SIGINT, self.stop)
    def start(self):
        try:
            pass
	    '''
            self._gripper_ser = serial.Serial(port="/dev/ttyUSB0",baudrate=115200,timeout=1,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
            data_raw = self._gripper_ser.readline()
            time.sleep(0.1)
            #get gripper status
            self._gripper_ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
            data_raw = self._gripper_ser.readline()
            time.sleep(0.1)
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
            data_raw = self._gripper_ser.readline()
            time.sleep(GRIPPER_DELAY)
            #self._gripper_ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
            #data_raw = self._gripper_ser.readline()
            #data = binascii.hexlify(data_raw)
            #status = data[12:14]
            #if status == "00":
            #    self._gripper_width = 0
            #elif status == "ff":
            #    self._gripper_width = 1
            #else:
            #    self._gripper_width = 0
            #    self.openGripper()
            #    time.sleep(GRIPPER_DELAY)
	    '''
        except:
	    print(sys.exc_info()[0])
            print("Warning, gripper not set up")


        data = open("path_data.txt", "w")
        data.close()
        # initialize RTDE
        self._conn = rtde.RTDE(self._robot_host, self._rtde_port)
        self._conn.connect()
        self._version = self._conn.get_controller_version()

        # configure outputs (URControl -> Python)
        self._conn.send_output_setup(
            ['timestamp', 'target_q', 'actual_q', 'target_qd', 'actual_qd', 'target_qdd', 'target_speed_fraction'],
            ['DOUBLE', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'VECTOR6D', 'DOUBLE']
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

        # start RTDE
        self._conn.send_start()

        # start the controller program
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._robot_host, self._command_port))
        program = _CONTROLLER_PROGRAM.format(tcp=self._tcp, payload=self._payload, gravity=self._gravity)
        logger.info('controller program:\n{}'.format(program))
        self._sock.sendall(program.encode('ascii') + b'\n')

        self._max_speed_scale = None

        controlThread = threading.Thread(target = self.controlLoop, args=[[target, setpoint_id, target_type, velocity, acceleration, lookahead, gain, speed_slider]])
        controlThread.start()

    def controlLoop(self, args):
        #signal.signal(signal.SIGINT, self.stop)
        #print("starting control loop")
        if not len(args) == 8:
            print("Error in thread configuration, should have 8 inputs")
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


        def r2l(reg, base=0):
            list = []
            for i in range(6):
                list.append(reg.__dict__['input_double_register_{}'.format(base + i)])
            return list

        def l2r(list, reg, base=0):
            for i in range (6):
                reg.__dict__['input_double_register_{}'.format(base + i)] = list[i]
            return reg

        self._quit = False

        setpoint_number = 0
        #could set this outside the thread, but it seems reasonable for when the thread starts the setpoint should be 1

        while not self._quit:
            #print(self._quit)

            # receive the current update_state

            state = self._conn.receive()
            #time.sleep(0.0001)
            #time.sleep(0.1)
            if state is None:
                logger.warn('lost RTDE connection -> stopping')
                break
            # honor speed scaling set when program started
            if self._max_speed_scale is None:
                self._max_speed_scale = state.target_speed_fraction
                self._speed_scale = state.target_speed_fraction

            # invoke update
            self.update(state)
            #self.update_state(state) ## Yifan's addition
            #used to be self.update_state(state)

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

        self._quit = True
        print("ending control loop")
        #if this loop exits, disconnect
        self._sock.sendall(b'stop program\n')

        self._conn.send_pause()
        self._conn.disconnect()
        print("disconnecting")


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

    def setMode(self, mode):
        if mode == 0 or mode == "discrete":
            #discrete mode
            self._mode = MODE_DISCRETE
        if mode == 1 or mode == "continuous":
            self._mode = MODE_CONTINUOUS

    def setSpeed(self, speed):
        if speed >= self._max_speed:
            self._speed = self._max_speed
        else:
            self._speed = speed

    def setMilestoneEps(self, eps):
        if eps >= self._max_eps:
            self._eps = self._max_eps
        else:
            self._eps = eps


    def setPath(self, path):
        self._pathLock.acquire()
        first = []
        for i in path:
            if isinstance(i, list):
                if len(i) != ROBOT_CONFIG_LEN:
                    print("Error, setPath needs milestones with ", ROBOT_CONFIG_LEN, " parameters")
                    return
            else:
                #path was a list of floats instead of a list of float lists
                print("Error, input path as a list of lists")
                return
        self._path = path
        self._pathLock.release()

    def appendPath(self, path):
        self._pathLock.acquire()
        for i in path:
            if isinstance(i, list):
                if len(i) == ROBOT_CONFIG_LEN:
                    self._path.append(i)
                elif len(i) == ROBOT_CONFIG_LEN-1:
                    if self._path:
                        i.append(self._path[-1][-1])
                    else:
                        i.append(1)
                        #default append open gripper
                    self._path.append(i)
            else:
                print("Error in path append formatting - not appending")
        self._pathLock.release()

    def getConfig(self):
        return self._q_curr

    def update(self,state):
        # added time into this
        if self._start_time is None:
            self._start_time = state.timestamp

        t = state.timestamp - self._start_time
        dt = t - self._last_t
        self._last_t = t

        self._q_curr=state.actual_q
        q_curr = self._q_curr
        q_curr.append(self._gripper_width)
        self._pathLock.acquire()
        #print("path is ", self._path)
        #check path is non-null

        if self._path:
            if(ur5_diff(self._path[0], q_curr)) <= self._eps/2 and self._last_t > self._gripper_delay_t:
                #only do gripper stuff once we've arrived at one of our waypoints
                #1 is open, 0 is closed
                if(self._path[0][-1] == 1):
                    self.openGripper()
                elif(self._path[0][-1] == 0):
                    self.closeGripper()
                #then update the paths
                self._path = self._path[1:]

        if self._path and self._last_t > self._gripper_delay_t :
            #check formatting of path[0]
            if isFormatted(self._path[0]) and isFormatted(q_curr) and inJointLimits(self._path[0] ) :
                #slope = vectorops.sub(self._path[0], q_curr)
                slope = []
                q_commanded = []
                for i in range(0, len(q_curr)):
                    slope.append(self._path[0][i] - q_curr[i])
                slope_norm=math.sqrt(ur5_diff(slope))

                if self._mode == MODE_DISCRETE:
                    q_commanded = self.discreteUpdate(q_curr, slope, slope_norm, dt)
                    #def discreteUpdate(self, q_curr, slope, slope_norm,  dt):
                elif self._mode == MODE_CONTINUOUS:
                    q_commanded = self.continuousUpdate(q_curr, slope, slope_norm, dt)

                self._q_commanded = [v for v in q_commanded]
            #we should do servo_position()
        else:
            #we should do servo_halt()
            if self._last_t < self._gripper_delay_t:
                print("Waiting for gripper")
            self._q_commanded = self._q_curr
        self._pathLock.release()

        #print('q_commanded',self._q_commanded)
        #Yifan's addition
        self.servo(q=self._q_commanded)
        #print('{:6.4f} {:6.4f}'.format(t, dt), ' '.join(['{:6.2f}'.format(x/pi*180) for x in state.actual_q]))
	data = open("path_data.txt", "a")
        #print('{:6.4f} {:6.4f}'.format(t, dt), ' '.join(['{:6.2f} '.format(x) for x in q_curr]))
	data.write('{:6.4f}'.format(t))
        for x in q_curr:
            data.write(" "+str(x))
        for xd in state.actual_qd:
            data.write(" "+str(xd))
        data.write('\n')
        data.close()

    def discreteUpdate(self, q_curr, slope, slope_norm, dt):
        q_commanded = []
        #if we are close enough, just move to the milestone
        if slope_norm <= self._eps:
            q_commanded = self._path[0]
        # if next step will hit milestone, slow down more
        elif slope_norm <= self._speed*dt:
            for i in range(0, len(q_curr)):
                q_commanded.append(0.5*slope[i]+q_curr[i])
        # normally just make sure our rate of change is within our desired value
        else:
            for i in range(0, len(q_curr)):
                q_commanded.append(self._speed*dt*slope[i]/slope_norm+q_curr[i])
        return q_commanded

    def continuousUpdate(self, q_curr, slope, slope_norm, dt):
        q_commanded = []
        #q_curr is 6 robot configs and one gripper config

        #last milestone or gripper is not what the next milestone is
        if (len(self._path) == 1) or not (q_curr[-1] == self._path[0][-1]):
            if slope_norm <= self._eps:
                q_commanded = self._path[0]
            elif slope_norm <= self._speed*dt:
                for i in range(0, len(q_curr)):
                    q_commanded.append(0.5*slope[i]+q_curr[i])
            else:
                for i in range(0, len(q_curr)):
                    q_commanded.append(self._speed*dt*slope[i]/slope_norm+q_curr[i])
        #other milestones
        else:
            if slope_norm <= self._speed*3*dt:
                self._path = self._path[1:]
                new_slope = []
                for i in range(0, len(q_curr)):
                    new_slope.append(self._path[0][i] - q_curr[i])
                new_slope_norm=math.sqrt(ur5_diff(new_slope))

                for i in range(0, len(q_curr)):
                    q_commanded.append(self._speed*dt*new_slope[i]/new_slope_norm+q_curr[i])
            else:
                for i in range(0, len(q_curr)):
                    q_commanded.append(self._speed*dt*slope[i]/slope_norm+q_curr[i])
        return q_commanded

    def closeGripper(self):
        #print "Close gripper"
        if(not self._gripper_ser):
            print("Error, gripper not enabled")
            return
        if(self._gripper_width == 1):
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
            #data_raw = self._gripper_ser.readline()
            self._gripper_width = 0
            self._gripper_delay_t = self._last_t + GRIPPER_DELAY

    def openGripper(self):
        #print "Open gripper"
        if(not self._gripper_ser):
            print("Error, gripper not enabled")
            return
        if(self._gripper_width ==0):
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
            #data_raw = self._gripper_ser.readline()
            self._gripper_width = 1
            self._gripper_delay_t = self._last_t + GRIPPER_DELAY

    def update_state(self, state):

        #goto q_desired
        pass

    @property
    def version(self):
        return self._version


if __name__ == '__main__':
    logging.basicConfig()

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot wiggle test', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')
    parser.add_argument('--frequency', '-f', type=float, help='wiggle frequency (Hz)', default=0.1)
    parser.add_argument('--amplitude', '-a', type=float, help='wiggle amplitude (deg)', default=5)

    args = parser.parse_args()
    joint_positions=[]
    data_file=open('calibration_joint_positions.txt','r')
    for line in data_file:
    	line=line.rstrip()
	l=[num for num in line.split(' ')]
	l2=[float(num) for num in l]
	joint_positions.append(l2)
    data_file.close()
    ctrl = Ur5Controller(args.robot, frequency=args.frequency, amplitude=args.amplitude)
    #print(ctrl._acceleration)
    ctrl.start()

    ctrl.setMode('discrete')
    ctrl.setSpeed(6)
    #ctrl.stop()

    print joint_positions
    #ctrl.setPath([[0,0,0,0,0,0,0],[0,0,-1.5,0,0,0,1]])
    ctrl.setPath([joint_positions[11]+[1.0] ])
    #ctrl.setPath([[0,-90.0/180.0*math.pi,0,-90.0/180.0*math.pi,0,0,1],[-5.82/180.0*math.pi,-117.99/180.0*math.pi,-92.96/180.0*math.pi,-10.96/180.0*math.pi,90.71/180.0*math.pi,52.92/180.0*math.pi,1],[-5.82/180.0*math.pi,-117.99/180.0*math.pi,-142.25/180.0*math.pi,-10.96/180.0*math.pi,90.71/180.0*math.pi,52.92/180.0*math.pi,1]])
    #ctrl.appendPath([[-5.76/180.0*math.pi,-144.65/180.0*math.pi,-137.21/180.0*math.pi,10.66/180.0*math.pi,90.61/180.0*math.pi,52.83/180.0*math.pi,0],[-5.82/180.0*math.pi,-112.83/180.0*math.pi,-142.12/180.0*math.pi,-16.24/180.0*math.pi,90.74/180.0*math.pi,52.93/180.0*math.pi,0],
    #[-41.34/180.0*math.pi,-111.79/180.0*math.pi,-144.48/180.0*math.pi,-15.29/180.0*math.pi,89.89/180.0*math.pi,7.84/180.0*math.pi,1]])
    while 1:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            ctrl.stop()
            break

    #signal.signal(signal.SIGINT, ctrl.stop)
    print('done')


    #\workspace\PyUniversalRobot-master\src\PyUniversalRobot\control>
