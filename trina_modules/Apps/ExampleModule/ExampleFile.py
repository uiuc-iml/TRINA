import time,math
import json
import os
import datetime
import csv
from threading import Thread,Lock
import sys
import signal
from Utils import TimedLooper

class ExampleSharedData:
	"""Use self.lock to manage any access to data if shared between threads.
	(This is not needed if your app only uses one thread.)
	"""
	def __init__(self):
		self.lock = Lock()
		self.robot = None
		self.status = None
		self.loopCount = None

class Example:
	def __init__(self,Jarvis = None, debugging = False):
		self.sharedData = ExampleSharedData()
		self.sharedData.robot = Jarvis
		self.sharedData.status = 'idle' #states are " idle, active"        
        self.sharedData.loopCount = 0

        main_thread = threading.Thread(target = self._infoLoop,args=(self.sharedData,))
		main_thread.daemon = True
        main_thread.start()

        other_thread = threading.Thread(target = self._other,args=(self.sharedData,))
		other_thread.daemon = True
        other_thread.start()
        self.threads = [main_thread,other_thread]

	def return_threads(self):
		return self.threads

	def return_processes(self):
		return []

	def _infoLoop(self,data):
		#With multiple threads, be very, very careful about accessing self! 
		looper = TimedLooper(0.1,name="infoLoop")
		while looper:
			with data.lock:
				data.robot.log_health()
				loop_start_time = time.time()
				status = data.robot.getActivityStatus()

				if(status == 'active'):
					if(data.status == 'idle'):
						print('\n\n\n\n starting up Example Module! \n\n\n\n\n')
						data.status = 'active'

				elif(status == 'idle'):
					if data.status == 'active':
						print('loop count =',data.loopCount)
						print('\n\n\n\n deactivating Example Module. \n\n\n\n\n')
						data.status = 'idle'

	def _otherLoop(self,data):
		#With multiple threads, be very, very careful about accessing self! 
		looper = TimedLooper(1.0,name="infoLoop")
		while looper:
			with data.lock:
				if data.status == 'active':
					data.loopCount += 1
					print("Robot commanded position:",data.robot.getKlamptCommandedPosition())

if __name__ == "__main__" :
    example = Example()
