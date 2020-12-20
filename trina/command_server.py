import time,math,datetime
import threading
from multiprocessing import Process, Manager, Pipe
import os,csv,sys,shlex
from threading import Thread,Lock
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import traceback
from multiprocessing import Pool, TimeoutError,Process
import sys, inspect
import atexit
import subprocess
import sys
import os
import klampt
from threading import Thread
if(sys.version_info[0] < 3):
	from future import *
else:
	from importlib import reload
import trina
from trina import jarvis
from trina.utils import TimedLooper,lock_wrap
import redis

class CommandServerJarvisHooks:
	"""Place here whatever access to the CommandServer object
	you believe that jarvis.APILayers should have.
	"""
	def __init__(self,server):
		self.switch_module_activity = lock_wrap(server.lock,server.switch_module_activity)
		self.deactivate_module = lock_wrap(server.lock,server.deactivate_module)
		self.activate_module = lock_wrap(server.lock,server.activate_module)
		self.restart_module = lock_wrap(server.lock,server.restart_module)
		

class CommandServer:
	def __init__(self,modules = []):
		# we first check if redis is up and running:
		try:
			self.server = trina.state_server.StateServer()
			print('Reem already up and running')
		except Exception as e:
			print("Reem server has not been created yet.  Try running from 'redrun redrun_config.json'")
			exit(1)
			# # if we cannot connect to redis, we start the server for once:
			# print('starting redis server because of ',e)
			# self.redis_process = Process(target = self.start_redis())
			# self.redis_process.daemon = False
			# self.redis_process.start()
			# # self.start_redis()
			# # wait for it to start
			# time.sleep(2)
			# # then we start our connections as normal:
			# self.interface = RedisInterface(host="localhost")
			# self.interface.initialize()
			# self.server = KeyValueStore(self.interface)

		# self.start_ros_stuff()
		# we then proceed with startup as normal
		self.always_active = set(['UI','MotionModule','Apps.DirectTeleoperation'])
		self.server['HEALTH_LOG'] = {}
		self.server['ACTIVITY_STATUS'] = {}
		self.module_command_env = {}
		#define the environment in which an API call should be run here
		self.module_command_env = {'command_server':CommandServerJarvisHooks(self)}
		self.startup = True
		self.shut_down_flag = False
		
		self.health_dict = {}
		# create the list of threads
		self.modules_dict = {}

		self.active_modules = dict()
		for i in self.always_active:
			self.active_modules[i] = True
		self.start_modules(modules,startup = True)

		self.lock = Lock()
		timeUpdaterThread = Thread(target=self.timeUpdaterLoop)
		timeUpdaterThread.daemon = True
		timeUpdaterThread.start()
		commandReceiverThread = Thread(target=self.commandReceiverLoop)
		commandReceiverThread.daemon = True
		commandReceiverThread.start()
		moduleMonitorThread = threading.Thread(target=self.moduleMonitorLoop)
		moduleMonitorThread.daemon = True
		moduleMonitorThread.start()
		atexit.register(self.shutdown_all)

	def start_module(self,module_class,name):
		#set up a new redis interface and trina queue
		module_server = trina.state_server.StateServer()
		#initialize APIs available to the module
		apis = dict()
		for i in self.modules_dict:
			try:
				api = i.api(name,module_server)
				apis[api._api_name] = api
			except NotImplementedError:
				pass
			except AttributeError:
				pass
		#set up special APIs here
		module_jarvis = jarvis.Jarvis(str(name),apis,module_server)
		self.server['ACTIVITY_STATUS'][name] = str('idle')
		a = module_class(module_jarvis)

		try:
			obj = a.moduleCommandObject()
			self.module_command_env[name] = obj
		except NotImplementedError:
			pass

		return a

	def start_modules(self,module_names = [],startup = False):
		from trina import modules
		from modules import apps
		activity_dict = {}
		command_dict = {}
		all_candidates = dict()
		for name, obj in inspect.getmembers(trina_modules):
			if not inspect.isclass(obj): continue
			if str(obj).find('trina_modules') == -1: continue
			all_candidates[name] = obj
		for name, obj in inspect.getmembers(Apps):
			if not inspect.isclass(obj): continue
			if str(obj).find('trina_modules') == -1: continue
			all_candidates['App_'+name] = obj
		if startup:
			if not module_names:
				print('\n\n Starting ALL modules available!')
				module_names = list(all_candidates.keys())
				#anything that starts with App should go at the end
				module_names = list(reversed(sorted(module_names)))
				print('Startup order:' + str(module_names) + '\n\n\n')
			else:
				print('\n\n Starting Only Modules:' + str(module_names) + '\n\n\n')
			for name in module_names:
				if name not in all_candidates:
					raise RuntimeError("Invalid module name {}, valid candidates {}".format(name,list(all_candidates.keys())))
				obj = all_candidates[name]
				try:
					tmp = self.start_module(obj,name)
				except Exception as e:
					print('Failed to initialize module',name,'due to ',e)
					traceback.print_exc()
					continue
				self.modules_dict[name] = tmp
				self.health_dict[name] = [True,time.time()]
				activity_dict[name] = 'idle'
				if(name not in self.always_active):
					self.active_modules[name] = False
				else:
					self.active_modules[name] = True

			self.server['HEALTH_LOG'] = self.health_dict
			self.server['ACTIVITY_STATUS'] = activity_dict
		else:
			print('Restarting only modules '+ str(module_names))
			for name in module_names:
				if name not in all_candidates:
					raise RuntimeError("Invalid module name {}, valid candidates {}".format(name,list(all_candidates.keys())))
				obj = all_candidates[name]
				print('killing module '+ name)
				self.modules_dict[name].terminate()
				self.modules_dict.update({name:[]})
				print('restarting module ' + name)
				try:
					tmp = self.start_module(obj,name)
				except Exception as e:
					print('Failed to initialize module',name,'due to ',e)
					traceback.print_exc()
					continue
				self.modules_dict.update({name:tmp})
				self.server['HEALTH_LOG'][name] = [True,time.time()]
				self.server['ACTIVITY_STATUS'][name] = 'idle'
				if(self.active_modules[name]):
					self.active_modules[name] = False

	def restart_module(self,name):
		"""App-accessible call."""
		if module in self.modules_dict:
			try:
				self.modules_dict[module].terminate()
			except Exception as e:
				traceback.print_exc(e)
				print("Problem terminating module",module,", restarting anyway")
		self.start_modules([name])
	
	def deactivate_module(self,name):
		"""App-accessible call."""
		if not self.active_modules[name]:
			return
		self.server['ACTIVITY_STATUS'][name] = 'idle'
		self.active_modules[name] = False
		if name in self.modules_dict:
			#it's managed by the CommandServer, deactivate access to other apis
			mod = self.modules_dict[name]
			if mod.jarvis.apis is not None:
				for apiname in mod.jarvis.apis:
					delattr(mod.jarvis,apiname)

	def activate_module(self,name):
		"""App-accessible call."""
		if self.active_modules[name]:
			return
		self.server['ACTIVITY_STATUS'][name] = 'active'
		self.active_modules[name] = True
		if name in self.modules_dict:
			#it's managed by the CommandServer, deactivate access to other apis
			mod = self.modules_dict[name]
			if mod.jarvis.apis is not None:
				for apiname,apihandle in mod.jarvis.apis.items():
					set(mod.jarvis,apiname,apihandle)

	def switch_module_activity(self,to_activate,to_deactivate = []):
		"""App-accessible call."""
		print('switching module activity:')
		if not to_deactivate:
			tmp = self.server['ACTIVITY_STATUS'].read()
			for i in tmp.keys():
				# print(i)
				if self.active_modules[str(i)]:
					self.deactivate_module(str(i))
		else:
			for i in to_deactivate:
				if self.active_modules[i]:
					self.deactivate_module(i)
		for i in to_activate:
			self.reactivate_module(i)

	def shutdown_all(self):
		self.shutdown()
		print('closing all and exiting')
		for module in self.modules_dict.keys():
			self.modules_dict[module].terminate()

	def timeUpdaterLoop(self):
		looper = TimedLooper(trina.settings.get('CommandServer.time_updater_dt'),name='timeUpdater')
		while looper:
			if self.shut_down_flag:
				break
			with self.lock:
				self.server['TRINA_TIME'] = time.time()
	
	def commandReceiverLoop(self):
		command_logger = CommandLogger('EXECUTED_COMMANDS')
		looper = TimedLooper(trina.settings.get('CommandServer.command_receiver_dt'),name='commandReceiver')
		trina_queue_reader = jarvis.RedisQueueReader(self.redis_server_ip)
		last_print_time = 0
		loop_counter = 0
		while looper:
			if self.shut_down_flag:
				break

			loop_counter +=1
			
			with self.lock:
				modules = self.active_modules.copy()
			for i,active in modules.items():
				module_commands = trina_queue_reader.read(str(i)+'_MODULE_COMMANDS')
				# print(i,":",robot_command)
				if module_commands:
					if active:
						for command in module_commands:
							self.run(command)
								# print(command)
						for command in module_commands:
							command_logger.log_command(command,time.time())
					else:
						print('ignoring commands from {} because it is inactive'.format(str(i)),module_commands)

			elapsedTime = looper.time_elapsed()
			if elapsedTime-last_print_time > 5:
				print('\nCommand receiver execution frequency = {} \n'.format(loop_counter/(elapsedTime - last_print_time)))
				loop_counter = 0
				last_print_time = elapsedTime

	def run(self,command):
		try:
			exec(command,self.module_command_env)
		except Exception as e:
			print('there was an error executing your command!',e)
		finally:
			print("command received was " + str(command))
			pass

	def moduleMonitorLoop(self):
		heartbeat_tolerance = trina.settings.get('CommandServer.heartbeat_tolerance')
		dt = trina.settings.get('CommandServer.health_monitor_dt')
		looper = TimedLooper(dt,name='moduleMonitor')
		while looper:
			if self.shut_down_flag:
				break

			#discover other modules run from command line
			with self.lock:
				modules = self.server['ACTIVITY_STATUS'].read()
				for key in modules:
					try:
						self.active_modules[str(key)]
					except KeyError as e:
						if(str(key) not in self.always_active):
							print("Discovered module",key,"run from command line... not managed by CommandServer")
							self.active_modules[str(key)] = False
						else:
							self.active_modules[str(key)] = True

				to_restart = []
				for module in self.modules_dict.keys():
					moduleStatus = self.server["HEALTH_LOG"][module].read()
					if (time.time()-moduleStatus[1]) > heartbeat_tolerance*dt:
						print("Module " + module + " is dead  due to timeout, queueing restart")
						to_restart.append(module)
					else:
						mod = self.modules_dict[module]
						if not mod.healthy():
							print("Module " + module + " is dead due to dead process, queueing restart")
							to_restart.append(module)
							break
			if to_restart:
				print('restarting modules ' + str(to_restart))
				for module in to_restart:
					try:
						self.modules_dict[module].terminate()
					except Exception as e:
						traceback.print_exc(e)
						print("Problem terminating module",module,", restarting anyway")
				self.start_modules(to_restart)

	def shutdown(self):
		#send shutdown to all modules
		self.shut_down_flag = True
		return 0

	def start_redis(self):
		"""Taken out in the Redrun version"""
		print('starting redis')
		origWD = os.getcwd() # remember our original working directory
		#setting up the start of the redis server
		redis_server_path = os.path.expanduser('~/database-server/redis-5.0.4/src/redis-server')
		redis_conf_path = os.path.expanduser('~/database-server/redis.conf')
		redis_folder = os.path.expanduser('~/database-server')
		command_string = '{} {}'.format(redis_server_path,redis_conf_path)
		os.chdir(redis_folder)
		args = shlex.split(command_string)

		if(sys.version_info[0] < 3):
			pid=os.fork()
			if pid == 0:
				try:
					os.setsid()
				except:
					print('could not separate the process.')
				# if pid==0: # new process
				self.redis_pipe = subprocess.Popen(args)
				while(True):
					time.sleep(1000)
		else:

			self.redis_pipe = subprocess.Popen(args,start_new_session = True)

		# reverting back to trina directory
		os.chdir(origWD)

	def start_ros_stuff(self):
		"""Taken out in the Redrun version"""
		print('starting ros stuff')
		origWD = os.getcwd() # remember our original working directory
		#setting up the start of the redis server
		# catkin_folder = os.path.expanduser('~/catkin_ws/devel/')
		# os.chdir(catkin_folder)
		# os.system('./setup.sh')

		command_string = 'roscore'
		gmapping_string = 'rosrun gmapping slam_gmapping scan:=base_scan _xmax:=10 _xmin:=-10 _ymax:=10 _ymin:=-10'
		ros_args = shlex.split(command_string)
		gmapping_args = shlex.split(gmapping_string)
		if(not rosgraph.is_master_online()):
			self.ros_process = subprocess.Popen(ros_args)
		else:
			print('roscore already running, skipping this part')
		time.sleep(3)

		self.gmapping = subprocess.Popen(gmapping_args)
		print('executed gmapping')
		os.chdir(origWD)


class CommandLogger(object):
	def __init__(self,key, host = 'localhost', port = 6379,max_length = 10000):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
		self.max_length = max_length
	def log_command(self,command,time):
		self.length = self.r.llen(self.key)
		if(self.length >= self.max_length):
			print('\n\n\n\n\nQUEUE OVERFLOW!!!! \n\n\n\n\n SOMETHING WRONG WITH THE LOGGER?')
		else:
			self.r.rpush(self.key,str([command,time]))



if __name__=="__main__":
	import argparse

	parser = argparse.ArgumentParser(description='Runs the Jarvis command server')
	parser.add_argument('--modules', default=['robot','App_DirectTeleOperation'], type=str, nargs='+', help='The list of modules to activate in trina_modules')
	args = parser.parse_args(sys.argv[1:])

	server = CommandServer(modules = args.modules)
	
	while(True):
		time.sleep(100)
		pass
