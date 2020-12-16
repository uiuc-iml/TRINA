import time,math
import json
import os
import datetime
import csv
from threading import Thread
import sys
import signal
from Utils import TimedLooper

class Example:
	def __init__(self,Jarvis = None, debugging = False):
		self.status = 'idle' #states are " idle, active"
		self.state = 'idle' #states are " idle, active
        self.robot = Jarvis

        main_thread = threading.Thread(target = self._infoLoop)
		main_thread.daemon = True
        main_thread.start()

	def return_threads(self):
		return [self._serveStateReciever, self._infoLoop]

	def return_processes(self):
		return []

	def _infoLoop(self):
		looper = TimedLooper(0.1,name="infoLoop")
		while looper:
			self.robot.log_health()
			loop_start_time = time.time()
			status = self.robot.getActivityStatus()

			if(status == 'active'):
				if(self.status == 'idle'):
					print('\n\n\n\n starting up Direct-Tele Operation Module! \n\n\n\n\n')
					self.status = 'active'
					self.state = 'active'

			elif(status == 'idle'):
				if self.status == 'active':
					self.state = 'idle'
					self.status = 'idle'


if __name__ == "__main__" :
    example = Example()
