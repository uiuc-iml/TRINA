#RobotController.py
from klampt.math import vectorops
from ur5_robotiq_controller import *
import ur5_config as UR5_CONSTANTS
from threading import Thread, Lock, RLock
import time
from copy import copy
from scipy import signal as scipysignal

class LimbController:
    def __init__(self, host, **kwargs):
        """
        - host: the UR5 controller IP address

        Keyword arguments:
        - gripper: whether gripper is enabled (True by default)

        UR5 keyword arguments
        - rtde_port: port for RTDE, default 30004
        - command_port: port for commands, default 30002
        #- tcp: tool center point in wrist frame (default 0,0,0.04,0,0,0)
        - cog: center of gravityfor payload (default 0,0,0) #the outward direction of the EE flange is z-axis, this is the UR5's EE frame
        - payload: estimated payload in kg
        - gravity: estimated gravity in base frame in N, default [0,0,9.81]
        """

        self.ur5 = UR5Controller(host,filters=[self._update],**kwargs)
        self._gripper = kwargs.get('gripper', False)
        self._type = kwargs.get('type','vacuum')

        self._start_time = None
        self._last_t = 0
        self._q_curr = []
        self._qdot_curr = []
        self._q_commanded = []
        self._qdot_commanded = []
        self._gravity = kwargs.pop('gravity', [0, 0, 9.82])
        self._wrench=[]
        self._started = False

        #Filter wrench
        self._wrench_offset = [0.0]*6
        #self._set_wrench_offset_flag = False
        self._filter_flag = True
        if self._filter_flag:

            self._filtered_wrench = []
            self._history_Fx = []
            self._history_Fy = []
            self._history_Fz = []
            self._history_tx = []
            self._history_ty = []
            self._history_tz = []
            self._history_length = 50

            ## filter parameters
            Wn=0.08
            [self.b,self.a]=scipysignal.butter(5,Wn,'low')

        #debugging
        self._speed_fraction=0.0
        self._min_joints = UR5_CONSTANTS.MIN_JOINTS
        self._max_joints = UR5_CONSTANTS.MAX_JOINTS
        self._min_velocities = UR5_CONSTANTS.MIN_VEL
        self._max_velocities = UR5_CONSTANTS.MAX_VEL
        self._command_lock = RLock()
        self._state_read = False

        self._new_gripper_action = False
        self._gripper_action = 1 #1 close 2 open

    def start(self):
        res = self.ur5.start()
        #wait for controller to initialize so that we can start in a valid config
        ##sleeping more because of the gripper activation
        if self._gripper:
            time.sleep(2.3)
        else:
            time.sleep(1.3)
        if res and self.ur5.running():
            current_config=self.getConfig()
            self.setConfig(current_config)
            self._started = True
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

    def getWrench(self,filtered = False):
        if filtered:
            return self._filtered_wrench
        else:
            return self._wrench

    def zeroFTSensor(self):
        """
        Currently this does not have any effect
        """
        self._command_lock.acquire()
        self._wrench_offset = copy(self._filtered_wrench)
        self._command_lock.release()

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
                print("limbController: Warning, config not set - outside of limits")

    def setVelocity(self, dq_in):
        if self.isFormatted(dq_in):
            if(self.inLimits(dq_in, self._min_velocities, self._max_velocities)):
                #do things
                self._command_lock.acquire()
                self._q_commanded = []
                self._qdot_commanded = dq_in
                self._command_lock.release()
            else:
                print("limbController: Warning, velocity not set - outside of limits")

    def openGripper(self):
        """
        Open the parallel gripper or release the vacuum gripper
        """
        if self._gripper:
            self._command_lock.acquire()
            self._new_gripper_action = True
            if self._type == 'vacuum':
                self._gripper_action = 2
            else:
                self._gripper_action = 1
            self._command_lock.release()
        else:
            print("limbController:gripper not enabled")

    def closeGripper(self):
        """
        Close the parallel gripper or start the vacuum gripper
        """
        if self._gripper:
            self._command_lock.acquire()
            self._new_gripper_action = True
            if self._type == 'vacuum':
                self._gripper_action = 1
            else:
                self._gripper_action = 2
            self._command_lock.release()
        else:
            print("limbController:gripper not enabled")

    def _update(self,state):
        self._state_read = False
        if self._start_time is None:
            self._start_time = state.timestamp

        t = state.timestamp - self._start_time
        dt = t - self._last_t
        self._last_t = t

        #update current notion of state
        q_curr = state.actual_q
        self._wrench=state.actual_TCP_force

        #Add and filter wrench here
        if self._filter_flag:
            if len(self._history_Fx) < self._history_length:
                self._history_Fx.append(self._wrench[0])
                self._history_Fy.append(self._wrench[1])
                self._history_Fz.append(self._wrench[2])
                self._history_tx.append(self._wrench[3])
                self._history_ty.append(self._wrench[4])
                self._history_tz.append(self._wrench[5])
                tmp_wrench = copy(self._wrench)
            else:
                assert len(self._history_Fx) == self._history_length
                assert len(self._history_Fy) == self._history_length
                assert len(self._history_Fz) == self._history_length
                assert len(self._history_tx) == self._history_length
                assert len(self._history_ty) == self._history_length
                assert len(self._history_tz) == self._history_length
                self._history_Fx[0:self._history_length - 1] = self._history_Fx[1:self._history_length]
                self._history_Fx[self._history_length - 1] = self._wrench[0]
                self._history_Fy[0:self._history_length - 1] = self._history_Fy[1:self._history_length]
                self._history_Fy[self._history_length - 1] = self._wrench[1]
                self._history_Fz[0:self._history_length - 1] = self._history_Fz[1:self._history_length]
                self._history_Fz[self._history_length - 1] = self._wrench[2]
                self._history_tx[0:self._history_length - 1] = self._history_tx[1:self._history_length]
                self._history_tx[self._history_length - 1] = self._wrench[3]
                self._history_ty[0:self._history_length - 1] = self._history_ty[1:self._history_length]
                self._history_ty[self._history_length - 1] = self._wrench[4]
                self._history_tz[0:self._history_length - 1] = self._history_tz[1:self._history_length]
                self._history_tz[self._history_length - 1] = self._wrench[5]
                #filtering all 6 of these takes about 0.1 ms
                tmp_wrench = [0.0]*6
                tmp_wrench[0] = scipysignal.lfilter(self.b,self.a,self._history_Fx)[self._history_length -1].tolist()
                tmp_wrench[1] = scipysignal.lfilter(self.b,self.a,self._history_Fy)[self._history_length -1].tolist()
                tmp_wrench[2] = scipysignal.lfilter(self.b,self.a,self._history_Fz)[self._history_length -1].tolist()
                tmp_wrench[3] = scipysignal.lfilter(self.b,self.a,self._history_tx)[self._history_length -1].tolist()
                tmp_wrench[4] = scipysignal.lfilter(self.b,self.a,self._history_ty)[self._history_length -1].tolist()
                tmp_wrench[5] = scipysignal.lfilter(self.b,self.a,self._history_tz)[self._history_length -1].tolist()
                # if not self._set_wrench_offset_flag:
                #     self._wrench_offset = copy(tmp_wrench)
                #     self._set_wrench_offset_flag = True

        self._command_lock.acquire()
        #filtered wrench
        if self._filter_flag:
            self._filtered_wrench = copy(tmp_wrench)
        self._speed_fraction=state.target_speed_fraction

        self._q_curr = q_curr
        qdot_curr = state.actual_qd
        self._qdot_curr = qdot_curr
        halt = None

        if self._q_commanded:
            if self.isFormatted(self._q_commanded):
                if not self.inLimits(self._q_commanded, self._min_joints, self._max_joints):
                    self._q_commanded = []
                    halt = 1
                    print("Warning, exceeding joint limits. Halting")
            else:
                self._q_commanded = []
                halt = 1
                print("Warning, improper position formatting. Halting")

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
                self._qdot_commanded = []
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

        #put q_commanded into a format that servo can use
        if self._q_commanded and not halt:
            #q_commanded includes the gripper, send to servo only the UR5 configuration
            #UR5_CL = ur5 configuration length
            servo_q_commanded = self._q_commanded[0:UR5_CONSTANTS.UR5_CL]
        elif self._qdot_commanded and not halt:
            servo_qd_commanded = self._qdot_commanded[0:UR5_CONSTANTS.UR5_CL]
        current_gravity = self._gravity

        #deal with the gripper
        if self._gripper:
            if self._new_gripper_action:
                if self._gripper_action == 1:
                    self.ur5.closeGripper()
                elif self._gripper_action == 2:
                    self.ur5.openGripper()
                self._new_gripper_action = False
        self._command_lock.release()
        self.ur5.servo(halt=halt, q=servo_q_commanded, qd=servo_qd_commanded, g=current_gravity)

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
            if len(val) == config.UR5_CONFIG_LEN:
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
    ur5 = LimbController(args.robot, gripper=True, gravity=[4.91,-4.91,-6.93672],type = 'parallel',payload =0.72,cog = [0,0,0.05])
    ur5.start()
    time.sleep(1)
    ur5.closeGripper()
    time.sleep(3.0)
    # ur5.openGripper()
    # time.sleep(2.0)
    # ur5.closeGripper()
    # time.sleep(3.0)
    ur5.stop()

    ###Test gripper
    # gripper = Robotiq2Controller()
    # gripper.start()
    # start_time=time.time()
    # while time.time()-start_time < 15:
    #     t=time.time()-start_time
    #     q7=abs(math.sin(0.5*t))
    #     gripper.command(q=[q7])
    #     time.sleep(0.005)
