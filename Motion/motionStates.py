class LimbState:
	def __init__(self):
		self.sensedq = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.commandedq = []
		self.commandedqQueue = []
		self.senseddq = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.commandeddq = []
		self.senseddqQueue = []
		self.sensedWrench = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.gravityVector = [0,0,-9.81]

		self.commandSent = True
		self.commandType = 0 #0 is position, 1 is velocity
		self.commandQueue = False
		self.lastCommandQueueTime = 0.0
		#for kinematic to use...
		self.lastSensedq = []

class mobileBase():
	def __init__(self):
		pass