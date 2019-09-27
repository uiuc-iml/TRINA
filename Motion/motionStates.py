class limbState:
	def __init__(self):
		self.sensedq = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.commandedq = []
		self.commandedqQueue = []
		self.senseddq = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.commandeddq = []
		self.senseddqQueue = []
		self.sensedWrench = [0.0,0.0,0.0,0.0,0.0,0.0]
		commandSent = True
		commandType = 0 #0 is position, 1 is velocity
		commandQueue = False
		lastCommandQueueTime = 0.0
