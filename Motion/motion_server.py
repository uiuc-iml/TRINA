from SimpleXMLRPCServer import SimpleXMLRPCServer
import signal
import sys
# from motion_logger import Motion #testing logs for motion.py
from motion import Motion

import logging
from datetime import datetime
filename = "errorLogs/logFile_" + datetime.now().strftime('%d%m%Y') + ".log"
logging.basicConfig(filename=filename,filemode='a',level=logging.DEBUG, format='motion_server: %(asctime)s - %(message)s',datefmt='%H:%M:%S')

global robot
global server_started
server_started = False


def _startServer(mode = "Kinematic", components = []):
	##global variable
	global robot
	global server_started
	robot = Motion(mode = mode,components = components)
	logging.info("%s mode is activated",robot.mode)
	server_started = True
	print("server started")
	return 0

def sigint_handler(signum,frame):
	global robot
	global server_started
	if server_started:
		robot.shutdown()
	sys.exit(0)

def _startup():
	global robot
	return robot.startup()

def _setLeftLimbPosition(q):
	global robot
	robot.setLeftLimbPosition(q)
	return 0

def _setRightLimbPosition(q):
	global robot
	robot.setRightLimbPosition(q)
	return 0

def _setLeftLimbPositionLinear(q,duration):
	global robot
	robot.setLeftLimbPositionLinear(q,duration)
	return 0

def _setRightLimbPositionLinear(q,duration):
	global robot
	robot.setRightLimbPositionLinear(q,duration)
	return 0

def _sensedLeftLimbPosition():
	global robot
	return robot.sensedLeftLimbPosition()

def _sensedRightLimbPosition():
	global robot
	return robot.sensedRightLimbPosition()

def _setLeftLimbVelocity(qdot):
	global robot
	robot.setLeftLimbVelocity(qdot)
	return 0

def _setRightLimbVelocity(qdot):
	global robot
	robot.setRightLimbVelocity(qdot)
	return 0

def _setLeftEEInertialTransform(Tgarget,duration):
	global robot
	return robot.setLeftEEInertialTransform(Tgarget,duration)


def _setRightEEInertialTransform(Tgarget,duration):
	global robot
	return robot.setRightEEInertialTransform(Tgarget,duration)


def _setLeftEEVelocity(v, tool):
	global robot
	robot.setLeftEEVelocity(v,tool)
	return 0

def _setRightEEVelocity(v, tool):
	global robot
	robot.setRightEEVelocity(v,tool)
	return 0

def _sensedLeftEETransform():
	global robot
	return robot.sensedLeftEETransform()

def _sensedRightEETransform():
	global robot
	return robot.sensedRightEETransform()

def _sensedLeftLimbVelocity():
	global robot
	return robot.sensedLeftLimbVelocity()

def _sensedRightLimbVelocity():
	global robot
	return robot.sensedRightLimbVelocity()

def _setBaseTargetPosition(q, vel):
	global robot
	robot.setBaseTargetPosition(q,vel)
	return 0

def _setBaseVelocity(q):
	global robot
	robot.setBaseVelocity(q)
	return 0

def _setTorsoTargetPosition(q):
	global robot
	robot.setTorsoTargetPosition(q)
	return 0

def _sensedBaseVelocity():
	global robot
	return robot.sensedBaseVelocity()

def _sensedBasePosition():
	global robot
	return robot.sensedBasePosition()

def _sensedTorsoPosition():
	global robot
	return robot.sensedTorsoPosition()

def _setLeftGripperPosition(position):
	global robot
	robot.setLeftGripperPosition(position)
	return 0

def _setLeftGripperVelocity(velocity):
	global robot
	robot.setLeftGripperVelocity(velocity)
	return 0

def _sensedLeftGripperPosition():
	global robot
	return robot.sensedLeftGripperPosition()

def _getKlamptCommandedPosition():
	global robot
	return robot.getKlamptSensedPosition()

def _getKlamptSensedPosition():
	global robot
	return robot.getKlamptSensedPosition()


def _shutdown():
	global robot
	robot.shutdown()
	return 0

def _isStarted():
	global robot
	return robot.isStarted()

def _isShutDown():
	global robot
	return robot.isShutDown()

def _moving():
	global robot
	return robot.moving()

def _stopMotion():
	global robot
	robot.stopMotion()
	return 0

def _resumeMotion():
	global robot
	robot.resumeMotion()
	return 0

def _mirror_arm_config(config):
	global robot
	return robot.mirror_arm_config(config)

def _getWorld():
	global robot
	return robot.getWorld()

def _cartesianDriveFail():
	global robot
	return robot.cartesianDriveFail()

def _sensedLeftEEVelocity(local_pt):
	global robot
	return robot.sensedLeftEEVelcocity(local_pt)

def _sensedRightEEVelocity(local_pt):
	global robot
	return robot.sensedRightEEVelcocity(local_pt)
#ip_address = 'localhost'
ip_address = '10.194.203.22'
#ip_address = '172.16.187.91'
#ip_address = '72.36.119.129'
port = 8080
server = SimpleXMLRPCServer((ip_address,port), logRequests=False)
server.register_introspection_functions()
signal.signal(signal.SIGINT, sigint_handler)

server.register_function(_startServer,'startServer')
##add functions...
server.register_function(_startup,'startup')
server.register_function(_setLeftLimbPosition,'setLeftLimbPosition')
server.register_function(_setRightLimbPosition,'setRightLimbPosition')
server.register_function(_setLeftLimbPositionLinear,'setLeftLimbPositionLinear')
server.register_function(_setRightLimbPositionLinear,'setRightLimbPositionLinear')
server.register_function(_sensedLeftLimbPosition,'sensedLeftLimbPosition')
server.register_function(_sensedRightLimbPosition,'sensedRightLimbPosition')
server.register_function(_setLeftLimbVelocity,'setLeftLimbVelocity')
server.register_function(_setRightLimbVelocity,'setRightLimbVelocity')
server.register_function(_setLeftEEInertialTransform,'setLeftEEInertialTransform')
server.register_function(_setRightEEInertialTransform,'setRightEEInertialTransform')
server.register_function(_setLeftEEVelocity,'setLeftEEVelocity')
server.register_function(_setRightEEVelocity,'setRightEEVelocity')
server.register_function(_sensedLeftEETransform,'sensedLeftEETransform')
server.register_function(_sensedRightEETransform,'sensedRightEETransform')
server.register_function(_sensedLeftLimbVelocity,'sensedLeftLimbVelocity')
server.register_function(_sensedRightLimbVelocity,'sensedRightLimbVelocity')
server.register_function(_setBaseTargetPosition,'setBaseTargetPosition')
server.register_function(_setBaseVelocity,'setBaseVelocity')
server.register_function(_setTorsoTargetPosition,'setTorsoTargetPosition')
server.register_function(_sensedBaseVelocity,'sensedBaseVelocity')
server.register_function(_sensedBasePosition,'sensedBasePosition')
server.register_function(_sensedTorsoPosition,'sensedTorsoPosition')
server.register_function(_setLeftGripperPosition,'setLeftGripperPosition')
server.register_function(_setLeftGripperVelocity,'setLeftGripperVelocity')
server.register_function(_sensedLeftGripperPosition,'sensedLeftGripperPosition')
server.register_function(_getKlamptCommandedPosition,'getKlamptCommandedPosition')
server.register_function(_getKlamptSensedPosition,'getKlamptSensedPosition')
server.register_function(_shutdown,'shutdown')
server.register_function(_isStarted,'isStarted')
server.register_function(_moving,'moving')
server.register_function(_stopMotion,'stopMotion')
server.register_function(_resumeMotion,'resumeMotion')
server.register_function(_mirror_arm_config,'mirror_arm_config')
server.register_function(_getWorld,'getWorld')
server.register_function(_cartesianDriveFail,'cartesianDriveFail')
server.register_function(_startup,'startup')
server.register_function(_isShutDown,'isShutDown')
server.register_function(_sensedLeftEEVelocity,'sensedLeftEEVelcocity')
server.register_function(_sensedRightEEVelocity,'sensedRightEEVelcocity')

##
print('#######################')
print('#######################')
logging.info("Server Created")
print('Server Created')
##run server
server.serve_forever()
