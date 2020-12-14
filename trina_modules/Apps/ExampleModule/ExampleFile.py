import time,math
import json
import os
import datetime
import csv
from threading import Thread
import sys
import signal


class Example:
	def __init__(self,Jarvis = None, debugging = False):
		self.status = 'idle' #states are " idle, active"
		self.state = 'idle' #states are " idle, active
        self.robot = Jarvis
		signal.signal(signal.SIGINT, self.sigint_handler) # catch SIGINT (ctrl+c)

        stateRecieverThread = threading.Thread(target=self._serveStateReciever)
		main_thread = threading.Thread(target = self._infoLoop)
        stateRecieverThread.start()
        main_thread.start()

	def sigint_handler(self, signum, frame):
		""" Catch Ctrl+C tp shutdown the api,
			there are bugs with using sigint_handler.. not used rn.

		"""
		assert(signum == signal.SIGINT)
		#logger.warning('SIGINT caught...shutting down the api!')
		print("SIGINT caught...shutting down the api!")

	def return_threads(self):
		return [self._serveStateReciever, self._infoLoop]

	def return_processes(self):
		return []

	def _infoLoop(self):
		while(True):
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

			elapsed_time = time.time() - loop_start_time
			if elapsed_time < self.infoLoop_rate:
				time.sleep(self.infoLoop_rate)
			else:
				time.sleep(0.001)

if __name__ == "__main__" :
    example = Example()
