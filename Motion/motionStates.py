class limbState:
	def __init__(self):
		self.sensedq = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.commandedq = []
		self.senseddq = [0.0,0.0,0.0,0.0,0.0,0.0]
		self.commandeddq = []
		self.sensedWrench = [0.0,0.0,0.0,0.0,0.0,0.0]
		commandSent = True
		commandType = 0 #0 is position, 1 is velocity
