import time
import sys
import time
from ..state_server import StateServer
from .api import APILayer

class Jarvis(APILayer):
	"""Currently implemented APIs are command_server (top level APIs), motion, and sensors.

	Usage:
		jarvis = [Jarvis instance, set up for you by command_server]
		components = jarvis.robot.components()
		mode = jarvis.robot.mode()
		jarvis.ui.getRayClick()
		etc...
	"""
	def __init__(self, name, apis=None, server=None):
		if server is None:
			from trina.modules.Motion import MotionAPI
			from trina.modules.UI import UIAPI
			#initialize
			server = StateServer()
			#initialize basic APIs available to the module
			apis = dict()
			apis['robot'] = MotionAPI('robot',name,server)
			apis['ui'] = UIAPI('ui',name,server)
			server.set(['ACTIVITY_STATUS',name],'idle')
		APILayer.__init__(self,'command_server',name,server)
		self.server = server
		self.name = name
		self.apis = apis
		if apis is not None:
			for apiname,apihandle in apis.items():
				assert apiname not in self.__dict__,"Can't define a module called "+apiname+" because it conflicts with an Jarvis attribute"
				setattr(self,apiname,apihandle)

	def log_health(self, status=True):
		"""Add the health log of the module
		"""
		self._redisSet(["HEALTH_LOG",self.name],[status, time.time()])

	def getActivityStatus(self):
		"""Returns the module activity 

		Return:
		--------------
		dict
		"""
		return self._redisGet(['ACTIVITY_STATUS',self.name])

	def changeActivityStatus(self, to_activate, to_deactivate=[]):
		"""Changes the status of the module
		"""
		self._moduleCommand('switch_module_activity', to_activate, to_deactivate)

	def restartModule(self, otherModule):
		self._moduleCommand('restart_module', otherModule)

	def getTrinaTime(self):
		"""Returns the trina time
		"""
		return self._redisGet(['TRINA_TIME'])

	def getUIState(self):
		""" Return UI state dictionary

		Return:
		----------------
		dict
		"""
		return self._redisGet(["UI_STATE"])

	def getRobotState(self):
		""" Return robot state dictionary

		Return:
		----------------
		dict
		"""
		return self._redisGet(["ROBOT_STATE"])

	def getSimulatedWorld(self):
		""" Return the simulated world

		Return:
		-------------
		The Klampt world of the simulated robot.
		"""
		return self._redisGet(["SIM_WORLD"])



class RedisQueue(object):
	"""Pushes from / reads from a module-specific queue on a Redis server.

	Thread-safe (redis clients are assumed thread safe).
	"""
	def __init__(self,key, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
	def push(self,item):
		self.r.rpush(self.key,item)

class RedisQueueReader(object):
	"""Write from a queue on a Redis server."""
	def __init__(self, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
	def read(self,key):
		with self.r.pipeline() as pipe:
			times = self.r.llen(key)
			for i in range(times):
				pipe.lpop(key)
			res = pipe.execute()
		return res
		

if __name__=="__main__":
	server = Jarvis('untitled')
