try:
	from SimpleXMLRPCServer import SimpleXMLRPCServer
except:
	from xmlrpc.server import SimpleXMLRPCServer
import signal
import sys
from motion import Motion
import time
import logging
from datetime import datetime
from trina_logging import get_logger
import numpy as np
filename = "errorLogs/logFile_" + datetime.now().strftime('%d%m%Y') + ".log"
logger = get_logger(__name__,logging.DEBUG,filename)


from xmlrpc.client import Marshaller
Marshaller.dispatch[np.float64] = Marshaller.dump_double
Marshaller.dispatch[np.ndarray] = Marshaller.dump_array

global robot
global server_started
server_started = False

ip_address = 'localhost'
port = 8080

def sigint_handler(signum,frame):
	global robot
	global server_started
	if server_started:
		robot.shutdown()
	sys.exit(0)

server = SimpleXMLRPCServer((ip_address,port), logRequests=False)
server.register_introspection_functions()
signal.signal(signal.SIGINT, sigint_handler)

import traceback
from functools import wraps

def xmlrpcMethod(name):
    """
    Decorator that registers a function to the xmlrpc server under the given name.
    """
    def register_wrapper(f):
        server.register_function(f, name)
        return f
    return register_wrapper

def loggedMethod(f):
    """
    Decorator that adds stack trace dumping (stdout and logger) to a function.
    """
    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except Exception as e:
            tb = traceback.format_exc()
            logger.error(tb)
            print(tb)
            raise e
    return wrapper

@loggedMethod
@xmlrpcMethod("startServer")
def _startServer(mode,components,codename):
	##global variable
	global robot
	global server_started

	if server_started:
		logger.info("server is already activated")
		print("server already started ")
	else:
		robot = Motion(mode = mode,components = components, codename = codename)
		logger.info("%s mode is activated",robot.mode)
		server_started = True
		print("server started")
	return 0

@loggedMethod
@xmlrpcMethod("restartServer")
def _restartServer(mode= "Kinematic", components = [] , codename = "seed"):
	global robot
	global server_started
	if(server_started):
		robot.shutdown()
	time.sleep(2)
	logger.info("Motion shutdown,restarting")
	robot = Motion(mode = mode,components = components, codename = codename)
	logger.info("%s mode is activated",robot.mode)
	server_started = True
	print("server started")
	return 0

@loggedMethod
@xmlrpcMethod("startup")
def _startup():
	global robot
	return robot.startup()

@loggedMethod
@xmlrpcMethod("setLeftLimbPosition")
def _setLeftLimbPosition(q):
	global robot
	robot.setLeftLimbPosition(q)
	return 0

@loggedMethod
@xmlrpcMethod("setRightLimbPosition")
def _setRightLimbPosition(q):
	global robot
	robot.setRightLimbPosition(q)
	return 0

@loggedMethod
@xmlrpcMethod("setLeftLimbPositionLinear")
def _setLeftLimbPositionLinear(q,duration):
	global robot
	robot.setLeftLimbPositionLinear(q,duration)
	return 0

@loggedMethod
@xmlrpcMethod("setRightLimbPositionLinear")
def _setRightLimbPositionLinear(q,duration):
	global robot
	robot.setRightLimbPositionLinear(q,duration)
	return 0

@loggedMethod
@xmlrpcMethod("sensedLeftLimbPosition")
def _sensedLeftLimbPosition():
	global robot
	return robot.sensedLeftLimbPosition()

@loggedMethod
@xmlrpcMethod("sensedRightLimbPosition")
def _sensedRightLimbPosition():
	global robot
	return robot.sensedRightLimbPosition()

@loggedMethod
@xmlrpcMethod("setLeftLimbVelocity")
def _setLeftLimbVelocity(qdot):
	global robot
	robot.setLeftLimbVelocity(qdot)
	return 0

@loggedMethod
@xmlrpcMethod("setRightLimbVelocity")
def _setRightLimbVelocity(qdot):
	global robot
	robot.setRightLimbVelocity(qdot)
	return 0

@loggedMethod
@xmlrpcMethod("setLeftEEInertialTransform")
def _setLeftEEInertialTransform(Tgarget,duration):
	global robot
	return robot.setLeftEEInertialTransform(Tgarget,duration)


@loggedMethod
@xmlrpcMethod("setRightEEInertialTransform")
def _setRightEEInertialTransform(Tgarget,duration):
	global robot
	return robot.setRightEEInertialTransform(Tgarget,duration)


@loggedMethod
@xmlrpcMethod("setLeftEEVelocity")
def _setLeftEEVelocity(v, tool):
	global robot
	robot.setLeftEEVelocity(v,tool)
	return 0

@loggedMethod
@xmlrpcMethod("setRightEEVelocity")
def _setRightEEVelocity(v, tool):
	global robot
	robot.setRightEEVelocity(v,tool)
	return 0

@loggedMethod
@xmlrpcMethod("sensedLeftEETransform")
def _sensedLeftEETransform(tool_center=[0,0,0]):
	global robot
	return robot.sensedLeftEETransform(tool_center=tool_center)

@loggedMethod
@xmlrpcMethod("sensedRightEETransform")
def _sensedRightEETransform(tool_center=[0,0,0]):
	global robot
	return robot.sensedRightEETransform(tool_center=[0,0,0])

@loggedMethod
@xmlrpcMethod("sensedLeftLimbVelocity")
def _sensedLeftLimbVelocity():
	global robot
	return robot.sensedLeftLimbVelocity()

@loggedMethod
@xmlrpcMethod("sensedRightLimbVelocity")
def _sensedRightLimbVelocity():
	global robot
	return robot.sensedRightLimbVelocity()

@loggedMethod
@xmlrpcMethod("setBaseTargetPosition")
def _setBaseTargetPosition(q, vel):
	global robot
	robot.setBaseTargetPosition(q,vel)
	return 0

@loggedMethod
@xmlrpcMethod("setBaseVelocity")
def _setBaseVelocity(q):
	global robot
	robot.setBaseVelocity(q)
	return 0

@loggedMethod
@xmlrpcMethod("setTorsoTargetPosition")
def _setTorsoTargetPosition(q):
	global robot
	robot.setTorsoTargetPosition(q)
	return 0

@loggedMethod
@xmlrpcMethod("sensedBaseVelocity")
def _sensedBaseVelocity():
	global robot
	return robot.sensedBaseVelocity()

@loggedMethod
@xmlrpcMethod("sensedBasePosition")
def _sensedBasePosition():
	global robot
	return robot.sensedBasePosition()

@loggedMethod
@xmlrpcMethod("sensedTorsoPosition")
def _sensedTorsoPosition():
	global robot
	return robot.sensedTorsoPosition()

@loggedMethod
@xmlrpcMethod("setLeftGripperPosition")
def _setLeftGripperPosition(position):
	global robot
	robot.setLeftGripperPosition(position)
	return 0

@loggedMethod
@xmlrpcMethod("setLeftGripperVelocity")
def _setLeftGripperVelocity(velocity):
	global robot
	robot.setLeftGripperVelocity(velocity)
	return 0

@loggedMethod
@xmlrpcMethod("sensedLeftGripperPosition")
def _sensedLeftGripperPosition():
	global robot
	return robot.sensedLeftGripperPosition()

@loggedMethod
@xmlrpcMethod("getKlamptCommandedPosition")
def _getKlamptCommandedPosition():
	global robot
	return robot.getKlamptSensedPosition()

@loggedMethod
@xmlrpcMethod("getKlamptSensedPosition")
def _getKlamptSensedPosition():
	global robot
	return robot.getKlamptSensedPosition()


@loggedMethod
@xmlrpcMethod("shutdown")
def _shutdown():
	global robot
	robot.shutdown()
	return 0

@loggedMethod
@xmlrpcMethod("pauseMotion")
def _pauseMotion():
	global robot
	robot.pauseMotion()
	return 0

@loggedMethod
@xmlrpcMethod("resumeMotion")
def _resumeMotion():
	global robot
	robot.resumeMotion()
	return 0

@loggedMethod
@xmlrpcMethod("isStarted")
def _isStarted():
	global robot
	return robot.isStarted()

@loggedMethod
@xmlrpcMethod("isShutDown")
def _isShutDown():
	global robot
	return robot.isShutDown()

@loggedMethod
@xmlrpcMethod("isPaused")
def _isPaused():
	global robot
	return robot.isPaused()

@loggedMethod
@xmlrpcMethod("moving")
def _moving():
	global robot
	return robot.moving()

@loggedMethod
@xmlrpcMethod("mirror_arm_config")
def _mirror_arm_config(config):
	global robot
	return robot.mirror_arm_config(config)

@loggedMethod
@xmlrpcMethod("getWorld")
def _getWorld():
	global robot
	return robot.getWorld()

@loggedMethod
@xmlrpcMethod("cartesianDriveFail")
def _cartesianDriveFail():
	global robot
	return robot.cartesianDriveFail()

@loggedMethod
@xmlrpcMethod("sensedLeftEEVelocity")
def _sensedLeftEEVelocity(local_pt):
	global robot
	return robot.sensedLeftEEVelocity(local_pt)

@loggedMethod
@xmlrpcMethod("sensedRightEEVelocity")
def _sensedRightEEVelocity(local_pt):
	global robot
	return robot.sensedRightEEVelocity(local_pt)

@loggedMethod
@xmlrpcMethod("sensedLeftEEWrench")
def _sensedLeftEEWrench(frame,tool_center = [0,0,0]):
	global robot
	return robot.sensedLeftEEWrench(frame,tool_center)

@loggedMethod
@xmlrpcMethod("sensedRightEEWrench")
def _sensedRightEEWrench(frame):
	global robot
	return robot.sensedRightEEWrench(frame)

@loggedMethod
@xmlrpcMethod("zeroLeftFTSensor")
def _zeroLeftFTSensor():
	global robot
	return robot.zeroLeftFTSensor()

@loggedMethod
@xmlrpcMethod("zeroRightFTSensor")
def _zeroRightFTSensor():
	global robot
	return robot.zeroRightFTSensor()

@loggedMethod
@xmlrpcMethod("openLeftRobotiqGripper")
def _openLeftRobotiqGripper():
	global robot
	return robot.openLeftRobotiqGripper()

@loggedMethod
@xmlrpcMethod("closeLeftRobotiqGripper")
def _closeLeftRobotiqGripper():
	global robot
	return robot.closeLeftRobotiqGripper()

@loggedMethod
@xmlrpcMethod("openRightRobotiqGripper")
def _openRightRobotiqGripper():
	global robot
	return robot.openRightRobotiqGripper()

@loggedMethod
@xmlrpcMethod("closeRightRobotiqGripper")
def _closeRightRobotiqGripper():
	global robot
	return robot.closeRightRobotiqGripper()

@loggedMethod
@xmlrpcMethod("setLeftEETransformImpedance")
def _setLeftEETransformImpedance(Tg,K,M,B,x_dot_g,deadband,tool_center):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setLeftEETransformImpedance(Tg,K,M,B,x_dot_g,deadband,tool_center)

@loggedMethod
@xmlrpcMethod("setRightEETransformImpedance")
def _setRightEETransformImpedance(Tg,K,M,B,x_dot_g,deadband,tool_center):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setRightEETransformImpedance(Tg,K,M,B,x_dot_g,deadband,tool_center)

@loggedMethod
@xmlrpcMethod("setLeftLimbPositionImpedance")
def _setLeftLimbPositionImpedance(q,K,M,B,x_dot_g,deadband,tool_center):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setLeftLimbPositionImpedance(q,K,M,B,x_dot_g,deadband,tool_center)

@loggedMethod
@xmlrpcMethod("setRightLimbPositionImpedance")
def _setRightLimbPositionImpedance(q,K,M,B,x_dot_g,deadband):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setRightLimbPositionImpedance(q,K,M,B,x_dot_g,deadband)	

@loggedMethod
@xmlrpcMethod("sensedHeadPosition")
def _sensedHeadPosition():
	global robot
	return robot.sensedHeadPosition()

@loggedMethod
@xmlrpcMethod("setHeadPosition")
def _setHeadPosition(q):
	global robot
	return robot.setHeadPosition(q)

#ip_address = '172.16.250.88'
# ip_address = '172.16.187.91'
#ip_address = '72.36.119.129'

# ip_address = '172.16.241.141'
# ip_address = '10.0.242.158'#'localhost' #'10.0.242.158'#

print('#######################')
print('#######################')
logger.info("Server Created")
print('Server Created')
##run server
server.serve_forever()
