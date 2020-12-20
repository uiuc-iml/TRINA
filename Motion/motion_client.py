import sys
if sys.version_info[0] >= 3:
	from xmlrpc.client import ServerProxy
else:
	from xmlrpclib import ServerProxy
from threading import Thread, Lock
import threading
import time
from klampt import WorldModel
import os
import numpy as np

try:
	import trina
except ImportError:
	sys.path.append(os.path.expandpath("~/TRINA"))
	import trina

class MotionClient:
	def __init__(self, address = 'auto'):
		if address == 'auto':
			address = trina.settings.motion_server_addr()
		self.s = ServerProxy(address)
		self.dt = 0.2
		self.shut_down = False
		#self.world = trina.settings.simulation_world_load()
		#self.robot = self.world.robot(0)

		#print("init complete")

	def _visualUpdateLoop(self):
		while not self.shut_down:
			q = self.getKlamptSensedPosition()
			self.robot.setConfig(q)
			time.sleep(self.dt)

	def startServer(self,mode,components,codename):
		self.s.startServer(mode,components,codename)

	def restartServer(self,mode=None,components=None,codename=None):
		if mode is not None:
			self.s.restartServer2(mode,components,codename)
		else:
			self.s.restartServer()

	def mode(self):
		return self.s.mode()

	def codename(self):
		return self.s.codename()

	def robotModel(self):
		return self.s.robotModel()

	def activeComponents(self):
		return self.s.activeComponents()

	def startup(self):
		res = self.s.startup()
		#print("startup called")
		#controlThread = threading.Thread(target = self._visualUpdateLoop)
		#controlThread.start()
		#print("sending startup")
		return res

	def setPosition(self,q):
		return 0

	def setLeftLimbPosition(self,q):
		self.s.setLeftLimbPosition(q)
		return 0

	def setRightLimbPosition(self,q):
		self.s.setRightLimbPosition(q)
		return 0

	def setLeftLimbPositionLinear(self,q,duration):
		self.s.setLeftLimbPositionLinear(q,duration)
		return 0

	def setRightLimbPositionLinear(self,q,duration):
		self.s.setRightLimbPositionLinear(q,duration)
		return 0

	def sensedLeftLimbPosition(self):
		return self.s.sensedLeftLimbPosition()

	def sensedRightLimbPosition(self):
		return self.s.sensedRightLimbPosition()

	def setVelocity(self,qdot):
		self.s.setVelocity(qdot)

	def setLeftLimbVelocity(self,qdot):
		self.s.setLeftLimbVelocity(qdot)

	def setRightLimbVelocity(self,qdot):
		self.s.setRightLimbVelocity(qdot)
		return

	def setLeftEEInertialTransform(self,Ttarget,duration):
		return self.s.setLeftEEInertialTransform(Ttarget,duration)


	def setLeftEEVelocity(self,v,tool):
		if not tool:
			tool = [0,0,0]
		self.s.setLeftEEVelocity(v,tool)

	def setRightEEInertialTransform(self,Ttarget,duration):
		return self.s.setRightEEInertialTransform(Ttarget,duration)


	def setRightEEVelocity(self, v ,tool):
		if not tool:
			tool = [0,0,0]
		self.s.setRightEEVelocity(v,tool)

	def sensedLeftEETransform(self):
		"""Return the transform w.r.t. the base frame"""
		return self.s.sensedLeftEETransform()

	def sensedRightEETransform(self):
		"""Return the transform w.r.t. the base frame"""
		return self.s.sensedRightEETransform()


	def sensedLeftLimbVelocity(self):
		return self.s.sensedLeftLimbVelocity()

	def sensedRightLimbVelocity(self):
		return self.s.sensedRightLimbVelocity()

	def setBaseTargetPosition(self, q, vel):
		self.s.setBaseTargetPosition(q,vel)

	def setBaseVelocity(self, q):
		self.s.setBaseVelocity(q)

	def setTorsoTargetPosition(self, q):
		self.s.setTorsoTargetPosition(q)

	# returns [v, w]
	def sensedBaseVelocity(self):
		return self.s.sensedBaseVelocity()

	# returns [x, y, theta]
	def sensedBasePosition(self):
		return self.s.sensedBasePosition()

	# returns [height, tilt]
	def sensedTorsoPosition(self):
		return self.s.sensedTorsoPosition()

	def setLeftGripperPosition(self, position):
		self.s.setLeftGripperPosition(position)

	def setLeftGripperVelocity(self,velocity):
		self.s.setLeftGripperVelocity(velocity)

	def sensedLeftGripperPosition(self):
		return self.s.sensedLeftGripperPosition()

	def getKlamptCommandedPosition(self):
		return self.s.getKlamptCommandedPosition()

	def getKlamptSensedPosition(self):
		return self.s.getKlamptSensedPosition()

	def shutdown(self):
		self.shut_down = True
		self.s.shutdown()

	def isStarted(self):
		return self.s.isStarted()

	def isShutDown(self):
		return self.s.isShutDown()

	def moving(self):
		"""Returns true if the robot is currently moving."""
		return self.s.moving()

	def mode(self):
		return self.s.mode()

	def stopMotion(self):
		self.s.stopMotion()

	def resumeMotion(self):
		self.s.resumeMotion()

	def mirror_arm_config(self,config):
		return self.s.mirror_arm_config(config)

	# def getWorld(self):
	# 	return self.world

	def cartesianDriveFail(self):
		return self.s.cartesianDriveFail()

	def sensedLeftEEVelocity(self,local_pt = [0,0,0]):
		return self.s.sensedLeftEEVelcocity(local_pt)

	def sensedRightEEVelocity(self,local_pt = [0,0,0]):
		return self.s.sensedRightEEVelcocity(local_pt)

	def sensedLeftEEWrench(self,frame = 'global'):
		return self.s.sensedLeftEEWrench(frame)

	def sensedRightEEWrench(self,frame = 'global'):
		return self.s.sensedRightEEWrench(frame)

	def zeroLeftFTSensor(self):
		return self.s.zeroLeftFTSensor()

	def zeroRightFTSensor(self):
		return self.s.zeroRightFTSensor()

	def openLeftRobotiqGripper(self):
		self.s.openLeftRobotiqGripper()

	def closeLeftRobotiqGripper(self):
		self.s.closeLeftRobotiqGripper()		

	def openRightRobotiqGripper(self):
		self.s.openRightRobotiqGripper()

	def closeRightRobotiqGripper(self):
		self.s.closeRightRobotiqGripper()	

	def setLeftEETransformImpedance(self,Tg,K,M,B,x_dot_g = [0]*6,deadband = [0]*6):
		K = K.tolist()
		B = B.tolist()
		M = M.tolist()
		self.s.setLeftEETransformImpedance(Tg,K,M,B,x_dot_g,deadband)

	def setRightEETransformImpedance(self,Tg,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6):
		K = K.tolist()
		B = B.tolist()
		M = M.tolist()
		self.s.setRightEETransformImpedance(Tg,K,M,B = B,x_dot_g = x_dot_g,deadband = deadband)

	def setLeftLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6):
		K = K.tolist()
		B = B.tolist()
		M = M.tolist()
		self.s.setLeftLimbPositionImpedance(q,K,M,B = B,x_dot_g = x_dot_g,deadband = deadband)

	def setRightLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6):
		K = K.tolist()
		B = B.tolist()
		M = M.tolist()
		self.s.setRightLimbPositionImpedance(q,K,M,B = B,x_dot_g = x_dot_g,deadband = deadband)


	def sensedHeadPosition(self):
		return self.s.sensedHeadPosition()

	def setHeadPosition(self,q):
		self.s.setHeadPosition(q)	

if __name__=="__main__":
	motion = MotionClient()
	motion.startup()
	time.sleep(0.05)
	try:
		print(motion.sensedLeftEETransform())
		time.sleep(0.05)
	except Exception as err:
		print("Error: ",str(err))
	motion.shutdown()
