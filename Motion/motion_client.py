import xmlrpclib
from threading import Thread, Lock
import threading
import time
from klampt import WorldModel
import os
dirname = os.path.dirname(__file__)
#getting absolute model name
model_name = os.path.join(dirname, "data/TRINA_world_reflex.xml")

class MotionClient:
	def __init__(self, address = 'http://localhost:8000'):
		self.s = xmlrpclib.ServerProxy('http://localhost:8000')
		self.dt = 0.2
		self.shut_down = False
		#self.world = WorldModel()
		#res = self.world.readFile(model_name)
		#if not res:
		#	raise RuntimeError("unable to load model")
		#self.robot = self.world.robot(0)

		#print("init complete")
	def _visualUpdateLoop(self):
		while not self.shut_down:
			q = self.getKlamptSensedPosition()
			self.robot.setConfig(q)
			time.sleep(self.dt)
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
		self.s.setLeftEEInertialTransform(Ttarget,duration)
		return

	def setLeftEEVelocity(self,v = None,w = None, tool = [0,0,0]):
		self.s.setLeftEEVelocity(v = v,w = w, tool = tool)

	def setRightEEInertialTransform(self,Ttarget,duration):
		self.s.setRightEEInertialTransform(Ttarget,duration)
		return

	def setRightEEVelocity(self,v = None,w = None, tool = [0,0,0]):
		self.s.setRightEEVelocity(v = v,w = w, tool = tool)


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

	def setGripperPosition(self, position):
		self.s.setGripperPosition(position)

	def setGripperVelocity(self,velocity):
		self.s.setGripperVelocity(velocity)

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

	def stopMotion(self):
		self.s.stopMotion()

	def resumeMotion(self):
		self.s.resumeMotion()

	def mirror_arm_config(self,config):
		return self.s.mirror_arm_config(config)

	def getWorld(self):
		return self.world

	def cartesianDriveFail(self):
		return self.s.cartesianDriveFail()

if __name__=="__main__":
	motion = MotionClient()
	motion.startup()
	print(motion.sensedLeftLimbPosition())
	motion.shutdown()