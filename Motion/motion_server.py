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

global robot
global server_started
global server_components

server_started = False

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

def _restartServer():
	global robot
	_restartServer2(robot.mode,robot.components,robot.codename)

def _restartServer2(mode= "Kinematic", components = [] , codename = "seed"):
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

def sigint_handler(signum,frame):
	global robot
	global server_started
	if server_started:
		robot.shutdown()
	sys.exit(0)

def _mode():
	global robot,server_started
	if not server_started:
		return ''
	return robot.mode

def _activeComponents():
	global robot,server_started
	if not server_started:
		return []
	res = []
	for part in ['left_limb','right_limb','base','torso','left_gripper','right_gripper','head']:
		if getattr(robot,part+'_enabled'):
			res.append(part)
	return res

def _codename():
	global robot,server_started
	if not server_started:
		return 'Anthrax'
	return robot.codename

def _robotModel():
	global robot,server_started
	if not server_started:
		return ''
	#format is ../Models/..., so strip the first 3 characters
	return robot.model_path[3:]

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
	return robot.sensedLeftEEVelocity(local_pt)

def _sensedRightEEVelocity(local_pt):
	global robot
	return robot.sensedRightEEVelocity(local_pt)

def _sensedLeftEEWrench(frame):
	global robot
	return robot.sensedLeftEEWrench(frame)

def _sensedRightEEWrench(frame):
	global robot
	return robot.sensedRightEEWrench(frame)

def _zeroLeftFTSensor():
	global robot
	return robot.zeroLeftFTSensor()

def _zeroRightFTSensor():
	global robot
	return robot.zeroRightFTSensor()

def _openLeftRobotiqGripper():
	global robot
	return robot.openLeftRobotiqGripper()

def _closeLeftRobotiqGripper():
	global robot
	print('left gripper called')
	return robot.closeLeftRobotiqGripper()

def _openRightRobotiqGripper():
	global robot
	return robot.openRightRobotiqGripper()

def _closeRightRobotiqGripper():
	global robot
	return robot.closeRightRobotiqGripper()

def _setLeftEETransformImpedance(Tg,K,M,B,x_dot_g,deadband):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setLeftEETransformImpedance(Tg,K,M,B,x_dot_g,deadband)

def _setRightEETransformImpedance(Tg,K,M,B,x_dot_g,deadband):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setRightEETransformImpedance(Tg,K,M,B,x_dot_g,deadband)

def _setLeftLimbPositionImpedance(q,K,M,B,x_dot_g,deadband):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setLeftLimbPositionImpedance(q,K,M,B,x_dot_g,deadband)

def _setRightLimbPositionImpedance(q,K,M,B,x_dot_g,deadband):
	global robot
	K = np.array(K)
	B = np.array(B)
	M = np.array(M)
	return robot.setRightLimbPositionImpedance(q,K,M,B,x_dot_g,deadband)	

def _sensedHeadPosition():
	global robot
	return robot.sensedHeadPosition()

def _setHeadPosition(q):
	global robot
	return robot.setHeadPosition(q)

def run_server_forever(ip_address,port):
	server = SimpleXMLRPCServer((ip_address,port), logRequests=False)
	server.register_introspection_functions()
	signal.signal(signal.SIGINT, sigint_handler)

	##add functions...
	server.register_function(_startServer,'startServer')
	server.register_function(_restartServer,'restartServer')
	server.register_function(_restartServer2,'restartServer2')
	server.register_function(_mode,'mode')
	server.register_function(_activeComponents,'activeComponents')
	server.register_function(_codename,'codename')
	server.register_function(_robotModel,'robotModel')
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
	server.register_function(_sensedLeftEEWrench,'sensedLeftEEWrench')
	server.register_function(_sensedRightEEWrench,'sensedRightEEWrench')
	server.register_function(_zeroLeftFTSensor,'zeroLeftFTSensor')
	server.register_function(_zeroRightFTSensor,'zeroRightFTSensor')
	server.register_function(_openLeftRobotiqGripper,'openLeftRobotiqGripper')
	server.register_function(_closeLeftRobotiqGripper,'closeLeftRobotiqGripper')
	server.register_function(_openRightRobotiqGripper,'openRightRobotiqGripper')
	server.register_function(_closeRightRobotiqGripper,'closeRightRobotiqGripper')
	server.register_function(_setLeftEETransformImpedance,'setLeftEETransformImpedance')
	server.register_function(_setRightEETransformImpedance,'setRightEETransformImpedance')
	server.register_function(_setLeftLimbPositionImpedance,'setLeftLimbPositionImpedance')
	server.register_function(_setRightLimbPositionImpedance,'setRightimbPositionImpedance')
	server.register_function(_setHeadPosition,'setHeadPosition')
	server.register_function(_sensedHeadPosition,'sensedHeadPosition')
	##
	print('#######################')
	print('#######################')
	logger.info("Server Created")
	print('Server Created')
	##run server
	server.serve_forever()

if __name__ == '__main__':
	import os,sys
	sys.path.append(os.path.expanduser("~/TRINA"))
	from Settings import trina_settings
	import argparse
	parser = argparse.ArgumentParser(description='Runs the motion server')
	parser.add_argument('-a','--ip', default='127.0.0.1', type=str, help='Server\'s IP address')
	parser.add_argument('-p','--port', default=trina_settings.motion_server_port(), type=int, help='Server\'s port number')
	parser.add_argument('--components', default=trina_settings.motion_server_components(), type=str, nargs='+', help='List of active components')
	parser.add_argument('--codename', default=trina_settings.robot_codename(), type=str, help="The robot model's codename")
	parser.add_argument('-m', '--mode', default=trina_settings.motion_server_mode(), type=str, help='The mode (Kinematic or Physical)')
	
	print()
	print("USAGE:")
	print()
	print("   motion_server.py [OPTIONS]")
	print()
	parser.print_help()
	args = parser.parse_args(sys.argv[1:])

	_startServer(args.mode,args.components,args.codename)
	run_server_forever(args.ip,args.port)