from Motion import MotionClient
from Motion import TRINAConfig
from .api import MotionAPI
import time
import trina
from trina import jarvis
from trina.utils import TimedLooper

class MotionModule(jarvis.APIModule):
    def __init__(self,Jarvis=None,robot_ip=None):
        jarvis.APIModule.__init__(self,Jarvis)
        self.status = 'active'
        if robot_ip is None:
            robot_ip = trina.settings.motion_server_addr()
        self.robot_ip = robot_ip
        self.robot = MotionClient(address = robot_ip)
        res = self.robot.startup()
        if not res:
            raise RuntimeError("Unable to connect to Motion Server")

        self.mode = self.robot.mode()
        self.codename = self.robot.codename()
        self.components = self.robot.activeComponents()
        self.left_limb_active = ('left_limb' in self.components)
        self.right_limb_active = ('right_limb' in self.components)
        self.base_active = ('base' in self.components)
        self.left_gripper_active = ('left_gripper' in self.components)
        self.right_gripper_active = ('right_gripper' in self.components)
        self.torso_active = ('torso' in self.components)

        self.pos_left = [0,0,0,0,0,0]
        self.pos_right = [0,0,0,0,0,0]
        self.pos_base = [0,0,0]
        self.pos_left_gripper = {}
        self.pos_right_gripper = {}
        self.pos_torso = {}
        self.vel_base = [0,0]
        self.vel_right = [0,0,0,0,0,0]
        self.vel_left = [0,0,0,0,0,0]
        self.posEE_left = {}
        self.velEE_left = {}
        self.posEE_right = {}
        self.velEE_right = {}

        print('\n\n\n\n\n\n\n Initializing robot states')
        self.robotRead = MotionClient(address = robot_ip)
        self.dt = trina.settings.app_settings('MotionModule')['state_receiver_dt']
        self.counter = 0
        #for some reason, the thread seems to run faster?
        self.startSimpleThread(self.stateReceiver_loop,self.dt,name='stateReceiver')
        #self.startProcess(self.stateReceiver_run)
        self.startMonitorThread(0.5)

    @classmethod
    def name(cls):
        return "Motion"

    @classmethod
    def apiClass(cls):
        return MotionAPI

    def moduleCommandObject(self):
        return self.robot

    def stateReceiver_run(self):
        self.robotRead = MotionClient(address = self.robot_ip)
        self.jarvis.server = trina.state_server.StateServer(self.jarvis.server)
        looper = TimedLooper(self.dt,name="MotionModule.stateReceiver")
        while looper:
            self.stateReceiver_loop()

    def stateReceiver_loop(self):
        self.counter += 1
        if self.counter%100 == 0:
            #periodically run the heartbeat
            self.jarvis.log_health()
        robot = self.robotRead
        server = self.jarvis.server
        try:
            #t0 = time.time()
            if(self.left_limb_active):
                self.posEE_left = robot.sensedLeftEETransform()
                self.pos_left = robot.sensedLeftLimbPosition()
                self.vel_left = robot.sensedLeftLimbVelocity()
                self.velEE_left = robot.sensedLeftEEVelocity()
                self.global_EEWrench_left = robot.sensedLeftEEWrench('global')
                self.local_EEWrench_left = robot.sensedLeftEEWrench('local')

            if(self.right_limb_active):
                self.posEE_right = robot.sensedRightEETransform()
                self.pos_right = robot.sensedRightLimbPosition()
                self.vel_right = robot.sensedRightLimbVelocity()
                self.velEE_right = robot.sensedRightEEVelocity()
                self.global_EEWrench_right = robot.sensedRightEEWrench('global')
                self.local_EEWrench_right = robot.sensedRightEEWrench('local')

            if(self.base_active):
                self.pos_base = robot.sensedBasePosition()
                self.vel_base = robot.sensedBaseVelocity()

            self.klampt_q = TRINAConfig.get_klampt_model_q(self.codename,left_limb = self.pos_left, right_limb = self.pos_right, base = self.pos_base)
            self.klampt_command_pos = robot.getKlamptCommandedPosition()
            self.klampt_sensor_pos = robot.getKlamptSensedPosition()
            if(self.left_gripper_active):
                self.pos_left_gripper = robot.sensedLeftGripperPosition()
            if(self.right_gripper_active):
                self.pos_right_gripper = robot.sensedRightGripperPosition()
            if(self.torso_active):
                self.pos_torso = robot.sensedTorsoPosition()

            #t1 = time.time()
            #print("Read robot info in time",t1-t0)

            server["ROBOT_INFO"] = {
                "Started" : robot.isStarted(),
                "Shutdown" : robot.isShutDown(),
                "Moving" : True,#robot.moving(),
                "CartesianDrive" : True,#robot.cartesianDriveFail(),
                "Components" : self.components,
                "Mode" : self.mode
            }
            server["ROBOT_STATE"] = {
                "Position" : {
                    "LeftArm" : self.pos_left,
                    "RightArm" : self.pos_right,
                    "Base" : self.pos_base,
                    "Torso": self.pos_torso,
                    "LeftGripper" : self.pos_left_gripper,
                    "RightGripper" : self.pos_right_gripper,
                    "Robotq": self.klampt_q
                },
                "PositionEE": {
                    "LeftArm" : self.posEE_left,
                    "RightArm" : self.posEE_right
                },
                "EEWrench":{
                    "LeftArm" :{
                        "global":self.global_EEWrench_left,
                        "local": self.local_EEWrench_left
                    },
                    "RightArm" :{
                        "global":self.global_EEWrench_right,
                        "local": self.local_EEWrench_right
                    }
                },
                "Velocity" : {
                    "LeftArm" : self.vel_left,
                    "RightArm" : self.vel_right,
                    "Base" : self.vel_base
                },
                "VelocityEE" : {
                    "LeftArm" : self.velEE_left,
                    "RightArm" : self.velEE_right
                },
                "KlamptCommandPos" : self.klampt_command_pos,
                "KlamptSensedPos" : self.klampt_sensor_pos
            }
            # print('states updated with success!')
        except Exception as e:
            print(e)
            import traceback
            traceback.print_exc()

