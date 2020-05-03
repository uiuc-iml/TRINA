#RobotController.py
from klampt.math import vectorops
from ur5_controller import *
import ur5_config as UR5_CONSTANTS
from threading import Thread, Lock
import CRC16
import time
import TRINAConfig.py

def addCRC(myHex):
    #takes a hex string and adds a modbus CRC to it
    crc = CRC16.calcString(myHex, CRC16.INITIAL_MODBUS)
    flipped_res = "{0:#0{1}x}".format(crc, 6)
    res = flipped_res[4:6] + flipped_res[2:4]
    add_on = res.decode('hex')
    newHex = myHex + add_on
    return newHex

class Robotiq2Controller:
    def __init__(self,port='/dev/ttyUSB0'):
        self.port = port
        self._gripper_ser = None
        #1 is open, 0 is closed

        #keeps track of the last read state of the gripper in case of gripper failure
        #happened occasionally was hard to determine cause
        self._last_gripper = 0
        #openGripper is 00
        #closeGripper is FF

    def start(self):
        try:
            self._gripper_ser = serial.Serial(port=self.port,baudrate=115200,timeout=0.005,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

            #sends a gripper command to initiate the gripper
            #self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
            start_command = "\x09\x10\x03\xE8\x00\x03\x06\x01\x00\x00\x00\x00\x00"
            start_command_with_crc = addCRC(start_command)
            self._gripper_ser.write(start_command_with_crc)

            data_raw = self._gripper_ser.readline()
            time.sleep(2)
            #sends a command to check the status
            # self._gripper_ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
            # data_raw =self._gripper_ser.readline()
            # time.sleep(1)
            # This command makes the gripper start in the open configuration on startup
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
            data_raw = self._gripper_ser.readline()
            time.sleep(1)

            # self._gripper_ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
            # data_raw = self._gripper_ser.readline()
            # time.sleep(0.1)
        except:
            print(sys.exc_info()[0])
            raise RuntimeError("Warning, gripper not set up")

    def command(self, q=None, qd=None, delay_t=UR5_CONSTANTS.DEFAULT_GRIPPER_DELAY):
        if sum([ x is not None for x in [q, qd]]) == 1:
            if(not self._gripper_ser):
                print("Error, gripper not enabled")
                return
            #we are doing either q or qd
            #            0   1   2   3   4   5   6   7   8   9  10  11  12
            command = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29"
            # default command for gripper                 position^   ^speed
            #            write to 3 registers^   ^6 bytes total     force^    ^CRC
            #00 is min speed, also min position (open)
            #FF is max speed, also max position (close)
            if q:
                if len(q) == 1:
                    gripper_pos = q[0]
                    #need to conver 0.0-1.0 to 3/255-230/255
                    gripper_pos = self._translate_virtual_to_physical(gripper_pos)
                    if gripper_pos <= 1 and gripper_pos >= 0:
                        gripper_command = chr(int(gripper_pos*255))
                        gripper_vel_command = "\xFF"
                        #if position is set, we set command to maximum velocity
                        command = command[0:10] + gripper_command + gripper_vel_command + command[12]
            elif qd:
                if len(qd) == 1:
                    gripper_vel = qd[0]
                    if abs(gripper_vel) <= 1 and abs(gripper_vel) > 0:
                        if(gripper_vel < 0):
                            #if vel negative, open (go towards 0)
                            gripper_vel_command = chr(int(abs(gripper_vel)*255))
                            gripper_command = chr(int(00))
                            command = command[0:10] + gripper_command + gripper_vel_command + command[12]
                        else:
                            #if vel positive, close (go towards 1)
                            gripper_vel_command = chr(int(abs(gripper_vel)*255))
                            gripper_command = chr(int(255))
                            command = command[0:10] + gripper_command + gripper_vel_command + command[12]
                    elif gripper_vel == 0:
                        #if vel is 0, stop gripper
                        gripper_vel_command = chr(int(00))
                        current_position = self._readGripper()
                        gripper_command = chr(int(current_position*255))
                        command = command[0:10] + gripper_command + gripper_vel_command + command[12]
                        #set gripper to "0"
                        #really set gripper to minspeed and stop moving
            else:
                #can return without setting gripper
                #if we get here, something has gone wrong
                print("Warning, neither q nor dq set for gripper")
                return
            #command for adding CRC is at the top of file
            #takes full command, appends CRC and returns appended command
            #Gripper needs CRC for error checking
            command = addCRC(command)
            self._gripper_ser.write(command)
            data = self._gripper_ser.read(8)

    #students don't use these functions. They interface with the robot through setConfig and setVelocity
    def read(self):
        #returns float 0-1
        #0 is open, 1 is closed
        if(not self._gripper_ser):
            print("Error, gripper not enabled")
            return 0
        #self._gripper_ser.flush()
        self._gripper_ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
        #this says read three registers starting from 07D0
        #response should be like:
        #09 03 06 E000 0000 0000 0000
        #         dat1 dat2 dat3 CRC
        #dat1 = Gripper Status, Object Detection,
        #dat2 = Fault Status and Position Request Echo
        #dat3 = Position, Current
        float_position = self._last_gripper
        data_raw = self._gripper_ser.readline()
        data = binascii.hexlify(data_raw)
        if len(data) < 16:
            position = data[14:16]
            try:
                int_position = int(position, 16)
                #translate back from 3/255-230/255 to 0.0-1.0
                float_position = self._translate_physical_to_virtual(int_position/255.0)
                self._last_gripper = float_position
                #sometimes, the gripper has trouble reading quickly
            except:
                print("gripper read exception")
                return float_position
        return float_position

    def _translate_physical_to_virtual(self, physical_value):
        #physical gripper can move 3/255 - 240/255
        #virtual gripper can move 0.0-1.0
        min_phys =UR5_CONSTANTS.MIN_GRIPPER_PHYSICAL
        max_phya =UR5_CONSTANTS.MAX_GRIPPER_PHYSICAL
        min_virt = UR5_CONSTANTS.MIN_JOINTS[UR5_CONSTANTS.GRIPPER_INDEX]
        max_virt = UR5_CONSTANTS.MAX_JOINTS[UR5_CONSTANTS.GRIPPER_INDEX]
        return (physical_value-min_phys)/(max_phys-min_phys) + min_virt

    def _translate_virtual_to_physical(self, virtual_value):
        min_phys =UR5_CONSTANTS.MIN_GRIPPER_PHYSICAL
        max_phys =UR5_CONSTANTS.MAX_GRIPPER_PHYSICAL
        min_virt = UR5_CONSTANTS.MIN_JOINTS[UR5_CONSTANTS.GRIPPER_INDEX]
        max_virt = UR5_CONSTANTS.MAX_JOINTS[UR5_CONSTANTS.GRIPPER_INDEX]
        return (virtual_value-min_virt)*(max_phys - min_phys) + min_phys


class LimbController:
    def __init__(self, host, **kwargs):
        """
        - host: the UR5 controller IP address

        Keyword arguments:
        - gripper: whether gripper is enabled (True by default)

        UR5 keyword arguments
        - rtde_port: port for RTDE, default 30004
        - command_port: port for commands, default 30002
        - tcp: tool center point in wrist frame (default 0,0,0.04,0,0,0)
        - payload: estimated payload in kg
        - gravity: estimated gravity in base frame in N, default [0,0,9.81]
        """
        self.ur5 = UR5Controller(host,filters=[self._update],**kwargs)
        if kwargs.pop('gripper', True):
            self.gripper = Robotiq2Controller()
        else:
            self.gripper = None

        self._start_time = None
        self._last_t = 0
        self._q_curr = []
        self._qdot_curr = []
        self._q_commanded = []
        self._qdot_commanded = []
        self._gravity = kwargs.pop('gravity', [0, 0, 9.82])
        self._wrench=[]
        self._started = False
        self.safety_status = -1 #-1 means the robot is not running, or disconnected

        #debugging
        self._speed_fraction=0.0
        self._min_joints = UR5_CONSTANTS.MIN_JOINTS
        self._max_joints = UR5_CONSTANTS.MAX_JOINTS
        self._min_velocities = UR5_CONSTANTS.MIN_VEL
        self._max_velocities = UR5_CONSTANTS.MAX_VEL
        self._command_lock = Lock()
        self._state_read = False

    def start(self):
        #start the gripper
        if self.gripper:
            self.gripper.start()

        res = self.ur5.start()
        time.sleep(0.2)
        #wait for controller to initialize so that we can start in a valid config
        current_config=self.getConfig()
        self.setConfig(current_config)
        # wait for the robot to initialize itself
        time.sleep(1)
        if res and self.ur5.running():
            self._started = True
            #return started running
            return True
        else:
            return False

    def stop(self):
        self.ur5.stop()

    ####
    def stopMotion(self):
        self.setConfig(self._q_curr)
        return

    def moving(self):
        return vectorops.norm(self._qdot_curr) > 0.0001
    ####

    def getVelocity(self):
        return self._qdot_curr

    def getConfig(self):
        return self._q_curr

    def getCurrentTime(self):
        return self._last_t

    def getSafetyStatus(self):
        if self.safety_status < 0:
            return -1,"not_connected"
        else:
            return self.safety_status, TRINAConfig.safety_status_names[self.safety_status]


    def getWrench(self):
        return self._wrench
    #FOR DEBUGGIN PURPOSES
    def getSpeedFraction(self):
        return self._speed_fraction

    def setGravity(self,g):
        if len(g) == 3 and vectorops.norm(g) < 10.0:
            self._gravity = g

    def setConfig(self, q_in):
        if self.isFormatted(q_in):
            if(self.inLimits(q_in, self._min_joints, self._max_joints)):
                self._command_lock.acquire()
                self._q_commanded = q_in
                self._qdot_commanded = []
                self._command_lock.release()
            else:
                print("Warning, config not set - outside of limits")

    def setVelocity(self, dq_in):
        if self.isFormatted(dq_in):
            if(self.inLimits(dq_in, self._min_velocities, self._max_velocities)):
                #do things
                self._command_lock.acquire()
                self._q_commanded = []
                self._qdot_commanded = dq_in
                self._command_lock.release()
            else:
                print("Warning, velocity not set - outside of limits")

    def _update(self,state):
        self._state_read = False
        if self._start_time is None:
            self._start_time = state.timestamp

        t = state.timestamp - self._start_time
        dt = t - self._last_t
        self._last_t = t

        #update current notion of state
        #update the safety Status
        safety_bits = state.safety_status_bits
        binary_bits = [int(x) for x in '{0:8b}'.format(safety_bits)]
        print(safety_bits,binary_bits)

        q_curr = state.actual_q
        q_gripper = (0 if self.gripper is None else self.gripper.read())
        self._wrench=state.actual_TCP_force
        self._speed_fraction=state.target_speed_fraction
        #change of gripper is about (current-previous)/dt
        #gripper does not provide velocity - only position. Velocity is estimated
        if self._q_curr:
            dq_gripper = (q_gripper - self._q_curr[-1])*1.0/dt
        else:
            #if current state is not existant (at the beginning) speed is 0
            dq_gripper = 0
        q_curr.append(q_gripper)
        self._q_curr = q_curr
        #q_curr is defined by gripper too
        #if gripper is not connected, gripper state is 0

        qdot_curr = state.actual_qd
        #qdot_curr = qdot_curr + gripperVel
        # if gripper is not connected, gripper velocity is 0
        qdot_curr.append(dq_gripper)
        self._qdot_curr = qdot_curr
        halt = None

        #check path is non-null

        #self._q_commanded is the commanded configuration that the students give. It has 7 parameters (6 for robot, 1 for gripper)
        self._command_lock.acquire()
        if self._q_commanded:
            #double extra check
            #if students are doing anything wrong
            if self.isFormatted(self._q_commanded):
                if not self.inLimits(self._q_commanded, self._min_joints, self._max_joints):
                    self._q_commanded = []
                    halt = 1
                    print("Warning, exceeding joint limits. Halting")
            else:
                halt = 1
                print("Warning, improper position formatting. Halting")
        #self._qdot_commanded is the commanded velocity that the students give. It has 7 parameters (6 for robot, 1 for gripper)
        if self._qdot_commanded:
            if self.isFormatted(self._qdot_commanded):
                if self.inLimits(self._qdot_commanded, self._min_velocities, self._max_velocities):
                    q_next = vectorops.madd(self._q_curr, self._qdot_commanded, dt)
                    #commanded velocity is rad/s
                    #only want to check next position limits of robot not gripper
                    #UR5_CL is the configuration length of just the UR5 robot = 6
                    if not self.inLimits(q_next[0:UR5_CONSTANTS.UR5_CL], self._min_joints[0:UR5_CONSTANTS.UR5_CL], self._max_joints[0:UR5_CONSTANTS.UR5_CL]):
                        self._qdot_commanded = []
                        halt = 1
                        print("Warning, exceeding joint limits. Halting")
            else:
                halt = 1
                print("Warning, improper velocity formatting. Halting")

        if not (self._q_commanded or self._qdot_commanded):
            #if neither position or velocity commands are set, go to the current position
            self._q_commanded = self._q_curr

        if (self._q_commanded and self._qdot_commanded):
            #if both position and velocity are set somehow, quit
            #this should never happen as the code stands - it's just in case
            halt = 1
            q_commanded = []
            qdot_commanded = []
            print("Error, don't set both q_commanded and qdot_commanded")

        servo_q_commanded = None
        servo_qd_commanded = None
        gripper_q_commanded = None
        gripper_qd_commanded = None

        #put q_commanded into a format that servo can use
        if self._q_commanded and not halt:
            #q_commanded includes the gripper, send to servo only the UR5 configuration
            #UR5_CL = ur5 configuration length
            servo_q_commanded = self._q_commanded[0:UR5_CONSTANTS.UR5_CL]
            gripper_q_commanded = [self._q_commanded[-1]]
        elif self._qdot_commanded and not halt:
            servo_qd_commanded = self._qdot_commanded[0:UR5_CONSTANTS.UR5_CL]
            gripper_qd_commanded = [self._qdot_commanded[-1]]
        current_gravity = self._gravity
        self._command_lock.release()

        #servo requires a list with six values
        #self._q_commanded and self._qd_commanded were previously checked for formatting
        #if halt is selected, the program on the controller terminates

        self.ur5.servo(halt=halt, q=servo_q_commanded, qd=servo_qd_commanded, g=current_gravity)
        if self.gripper is not None:
            #gripper.command requires a list of one value like -> [0]
            self.gripper.command(q=gripper_q_commanded, qd=gripper_qd_commanded)

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
        return self.ur5.running()

    def connected(self):
        return self.ur5.connected()

    def markRead(self):
        self._state_read = True
        return 0

    def newState(self):
        return (not self._state_read)
if __name__ == "__main__":

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot wiggle test', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')
    parser.add_argument('-g', '--gripper', type=bool, help='enable gripper', default=True)

    args = parser.parse_args()
    ur5 = LimbController(args.robot, gripper=False, gravity=[0,0,9.82])
    ur5.start()
    time.sleep(1)

    start_time=time.time()
    while time.time()-start_time < 2:
        # t=time.time()-start_time
        # q1=0.3*math.sin(t/0.5)
        # q3=0.3*math.sin(t/0.5)
        # q7=abs(math.sin(0.5*t))
        # position = [q1,-math.pi/2,q3,-math.pi/2,0,0,0]
        # ur5.setConfig(position)
        #print ur5.getCurrentTime()
        #print ur5.getWrench()
        time.sleep(0.01)

    ur5.stop()

    # gripper = Robotiq2Controller()
    # gripper.start()
    # start_time=time.time()
    # while time.time()-start_time < 15:
    #     t=time.time()-start_time
    #     q7=abs(math.sin(0.5*t))
    #     gripper.command(q=[q7])
    #     time.sleep(0.005)
