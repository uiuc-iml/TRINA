from xmlrpc.client import ServerProxy
from threading import Thread, Lock
import threading
import time
from klampt import WorldModel
import os
import numpy as np
dirname = os.path.dirname(__file__)
#getting absolute model name
model_name = os.path.join(dirname, "data/TRINA_world_seed.xml")

class MotionClient:
	def __init__(self, address = 'http://127.0.0.1:8080'):
		self.s = ServerProxy(address)
		self.dt = 0.2
		self.shut_down = False
		# self.world = WorldModel()
		# res = self.world.readFile(model_name)
		# if not res:
		# 	raise RuntimeError("unable to load model")
		# self.robot = self.world.robot(0)

		print("init complete")

	def startServer(self,mode,components,codename):
		self.s.startServer(mode,components,codename)

	def restartServer(self,mode,components,codename):
		self.s.restartServer(mode,components,codename)

	def startup(self):
		res = self.s.startup()
		print("startup called")
		# controlThread = threading.Thread(target = self._visualUpdateLoop)
		# controlThread.start()
		print("sending startup")
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

	def sensedLeftEETransform(self, tool_center=[0,0,0]):
		"""Return the transform w.r.t. the base frame"""
		return self.s.sensedLeftEETransform(tool_center)

	def sensedRightEETransform(self, tool_center=[0,0,0]):
		"""Return the transform w.r.t. the base frame"""
		return self.s.sensedRightEETransform(tool_center)


	def sensedLeftLimbVelocity(self):
		return self.s.sensedLeftLimbVelocity()

	def sensedRightLimbVelocity(self):
		return self.s.sensedRightLimbVelocity()

	def setBaseTargetPosition(self, q, vel):
		self.s.setBaseTargetPosition(q,vel)

	def setBaseVelocity(self, q):
		self.s.setBaseVelocity(q)

	def setBaseVelocityRamped(self,q,time):
		self.s.setBaseVelocityRamped(q,time)

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

	def sensedGripperPosition(self):
		return self.s.sensedGripperPosition()

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

	def pauseMotion(self):
		self.s.pauseMotion()

	def resumeMotion(self):
		self.s.resumeMotion()

	def isPaused(self):
		self.s.isPaused()

	def mirror_arm_config(self,config):
		return self.s.mirror_arm_config(config)

	# def getWorld(self):
	# 	return self.world

	def cartesianDriveFail(self):
		return self.s.cartesianDriveFail()

	def sensedLeftEEVelocity(self,local_pt = [0,0,0]):
		return self.s.sensedLeftEEVelocity(local_pt)

	def sensedRightEEVelocity(self,local_pt = [0,0,0]):
		return self.s.sensedRightEEVelocity(local_pt)


	def sensedLeftEEWrench(self,frame = 'global',tool_center = [0,0,0]):
		return self.s.sensedLeftEEWrench(frame,tool_center)

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

	def setLeftEETransformImpedance(self,Tg,K,M,B,x_dot_g = [0]*6,deadband = [0]*6,tool_center = [0.0]*3,col_mode = False):
		# K = K.tolist()
		# B = B.tolist()
		# M = M.tolist()
		self.s.setLeftEETransformImpedance(Tg,K,M,B,x_dot_g,deadband,tool_center,col_mode)

	def setRightEETransformImpedance(self,Tg,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,tool_center = [0.0]*3,col_mode = False):
		# K = K.tolist()
		# B = B.tolist()
		# M = M.tolist()
		self.s.setRightEETransformImpedance(Tg,K,M,B,x_dot_g,deadband,tool_center,col_mode)

	def setLeftLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,col_mode = False):
		K = K.tolist()
		B = B.tolist()
		M = M.tolist()
		self.s.setLeftLimbPositionImpedance(q,K,M,B,x_dot_g,deadband,col_mode)

	def setRightLimbPositionImpedance(self,q,K,M,B = np.nan,x_dot_g = [0]*6,deadband = [0]*6,col_mode = False):
		K = K.tolist()
		B = B.tolist()
		M = M.tolist()
		self.s.setRightLimbPositionImpedance(q,K,M,B,x_dot_g,deadband,col_mode)

	def sensedHeadPosition(self):
		return self.s.sensedHeadPosition()

	def setHeadPosition(self,q):
		self.s.setHeadPosition(q)
		
if __name__=="__main__":
	motion = MotionClient('http://localhost:8080')
	motion.startServer(mode = "Physical", components = ['right_limb','left_limb'], codename = 'cholera')
	motion.startup()
	time.sleep(0.05)
	T = motion.sensedRightEETransform()
	motion.setRightEEInertialTransform(T, 1)
	time.sleep(1)
	motion.shutdown()
