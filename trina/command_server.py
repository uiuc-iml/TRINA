import time,math,datetime
import threading
from multiprocessing import Process, Manager, Pipe
import os,csv,sys,shlex
from threading import Thread,Lock
import traceback
from multiprocessing import Pool, TimeoutError,Process
import sys, inspect
import atexit
import subprocess
import sys
import os
import klampt
from threading import Thread
if sys.version_info[0] < 3:
    from future import *
else:
    from importlib import reload
from importlib import import_module
try:
    import trina
except ImportError:  #must be run from command line
    sys.path.append(os.path.expanduser("~/TRINA"))
    import trina
from trina import jarvis
from trina.utils import TimedLooper,lock_wrap
import redis

class CommandServerJarvisHooks:
    """Place here whatever access to the CommandServer object
    you believe that jarvis.APILayers should have.
    """
    def __init__(self,server):
        self.switch_module_activity = server.switch_module_activity
        self.deactivate_module = server.deactivate_module
        self.activate_module = server.activate_module
        self.restart_module = server.restart_module

class RunningModuleInfo:
    def __init__(self,name,modobj=None,classobj=None,classinstance=None):
        self.name = name,
        self.modobj = modobj
        self.classobj = classobj
        self.classinstance = classinstance
        try:
            self.apiclass = classobj.apiClass()
            self.apiname = classobj.apiClass().name()
        except Exception:
            self.apiclass = None
            self.apiname = None
        self.jarvis = None if classinstance is None else classinstance.jarvis
        self.active = True

    def is_managed(self):
        return self.modobj is not None


class CommandServer:
    def __init__(self,modules = []):
        self.verbose = trina.settings.app_settings('CommandServer')['verbose']
        # we first check if redis is up and running:
        try:
            self.server = trina.state_server.StateServer()
            if self.verbose:
                print('Reem already up and running')
        except Exception as e:
            print(e)
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
            # self.server = trina.state_server.StateServer()

        # self.start_ros_stuff()
        # we then proceed with startup as normal

        self.server['HEALTH_LOG'] = {}
        self.server['ACTIVITY_STATUS'] = {}
        self.module_command_env = {}
        self.lock = Lock()
        #define the environment in which an API call should be run here
        self.module_command_env = {'jarvis':CommandServerJarvisHooks(self)}
        self.startup = True
        self.shut_down_flag = False
        
        # a map for module names (brief CapWords descriptor) to RunningModuleInfo
        self.modules_dict = {}

        self.start_modules(modules)

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

    def create_jarvis(self,name,server=None):
        #initialize APIs available to the module
        if server is None:
            server = trina.state_server.StateServer()
        apis = dict()
        for i,mod in self.modules_dict.items():
            try:
                api = mod.classinstance.api(name,server)
                apis[api.name()] = api
            except NotImplementedError as e:
                print("Module",i,"has no API",e)
                pass
            except AttributeError as e:
                print("Module",i,"has no API",e)
                pass
        if self.verbose:
            print("Available APIs for module",name,":",list(apis.keys()))
        res = jarvis.Jarvis(str(name),apis,server)
        #take out APIs for deactivated modules from the Jarvis namespace
        for i,mod in self.modules_dict.items():
            if not mod.active and mod.apiname is not None:
                delattr(res,mod.apiname)
        return res

    def start_module(self,module_class,name):
        """Starts up a new module and returns the class instance"""
        #set up a new redis interface and trina queue
        module_server = trina.state_server.StateServer()
        module_jarvis = self.create_jarvis(name,module_server)
        self.server['HEALTH_LOG'][name] = [True,time.time()]
        self.server['ACTIVITY_STATUS'][name] = 'waiting'
        a = module_class(module_jarvis)
        self.server['ACTIVITY_STATUS'][name] = a.status
        debugModules = trina.settings.app_settings('CommandServer')['debug']
        if name in debugModules:
            #turn on debugging
            a.verbose = max(1,a.verbose)
        silentModules = trina.settings.app_settings('CommandServer')['silent']
        if name in silentModules:
            a.verbose = 0

        #determine whether this module implements the moduleCommand interface
        try:
            obj = a.moduleCommandObject()
            self.module_command_env[a.apiClass().name()] = obj
            if self.verbose:
                print("Module command env",list(self.module_command_env.keys()))
        except NotImplementedError:
            pass
        except AttributeError:
            pass
        return a

    def start_dummy_module(self,name):
        """Creates an unmanaged dummy module and returns the class instance"""
        with self.lock:
            if name in self.modules_dict:
                raise ValueError("Module {} already exists".format(name))
            self.modules_dict[name] = RunningModuleInfo(name)

        module_server = trina.state_server.StateServer()
        module_jarvis = self.create_jarvis(name,module_server)
        mod = RunningModuleInfo(name)
        mod.jarvis = module_jarvis
        mod.active = True
        self.server['HEALTH_LOG'][name] = ['off',0]
        self.server['ACTIVITY_STATUS'][name] = 'active'
        #module_jarvis.enableHealthChecks(False)
        with self.lock:
            self.modules_dict[name] = mod
        return module_jarvis

    def start_modules(self,module_names = []):
        activity_dict = {}
        command_dict = {}
        if not module_names:
            import trina.modules
            import trina.modules.apps
            import trina_devel.modules
            import trina_devel.modules.apps
            all_modules = dict()
            for name, obj in inspect.getmembers(trina.modules) + inspect.getmembers(trina_devel.modules):
                if not inspect.isclass(obj): continue
                if str(obj).find('modules') == -1: continue
                all_modules[name] = obj
            for name, obj in inspect.getmembers(trina.modules.apps) + inspect.getmembers(trina_devel.modules.apps):
                if not inspect.isclass(obj): continue
                if str(obj).find('modules') == -1: continue
                all_modules['App_'+name] = obj
            print('\n\n Starting ALL modules available!')
            module_names = list(all_modules.keys())
            #anything that starts with App should go at the end
            module_names = list(reversed(sorted(module_names)))
            print('Startup order:' + str(module_names) + '\n\n\n')
        newmods = [name for name in module_names if name not in self.modules_dict]
        restartmods = [name for name in module_names if name in self.modules_dict]
        if newmods:
            print('\n\n Starting New Modules:' + str(newmods) + '\n\n\n')
        if restartmods:
            print('\n\n Restarting Modules:' + str(restartmods) + '\n\n\n')
        
        if newmods:
            import trina
            module_locations = trina.settings.app_settings('Jarvis')['module_locations']
            app_locations = trina.settings.app_settings('Jarvis')['app_locations']
            mod_class_map = trina.settings.app_settings('Jarvis')['module_class_map']

        module_updates = dict()
        for name in module_names:
            briefName = name  #this is the name used by the Python module
            if name.startswith('App_'):
                briefName = name[4:]
            if name not in self.modules_dict:  #new modules
                print("  Starting module",name,"...")
                modobj = None
                if name.startswith('App_'):
                    for location in app_locations:
                        try:
                            modobj = import_module(location+'.'+briefName)
                        except ImportError:
                            pass
                else:
                    for location in module_locations:
                        try:
                            modobj = import_module(location+'.'+briefName)
                        except ImportError:
                            pass
                if modobj is None:
                    if name.startswith("App_"):
                        raise RuntimeError("Invalid app name {}, could not be found/loaded in {}".format(briefName,app_locations))
                    else:
                        raise RuntimeError("Invalid module name {}, could not be found/loaded in {}".format(briefName,module_locations))
                modclassname = mod_class_map.get(name,briefName)
                try:
                    modclass = getattr(modobj,modclassname)
                except AttributeError:
                    raise RuntimeError("Class {} not found in module {}; perhaps you should set the entry point in settings.Jarvis.module_class_map?".format(modclassname,modobj.__file__))
                try:
                    classinstance = self.start_module(modclass,name)
                except Exception as e:
                    print('Failed to initialize module',name,'due to ',e)
                    traceback.print_exc()
                    print("\n\n")
                    continue
                print("\n\n")
                mod = RunningModuleInfo(name,modobj,modclass,classinstance)
                mod.active = (classinstance.status != 'idle')
                module_updates[name] = mod
            else: #existing module, restart
                print('   Killing module '+ name)
                mod = self.modules_dict[name]
                mod.active = False
                mod.classinstance.terminate()
                print('   Restarting module ' + name)
                #need to reload module and all sub-modules
                reload(mod.modobj)
                for k in list(sys.modules):
                    if k.startswith(mod.modobj.__name__):
                        reload(sys.modules[k])
                #reload class map
                import trina
                trina.settings.reload()
                mod_class_map = trina.settings.app_settings('Jarvis')['module_class_map']
                modclassname = mod_class_map.get(name,briefName)
                try:
                    mod.classobj = getattr(mod.modobj,modclassname)
                except:
                    raise RuntimeError("Class {} not found in module {}; perhaps you should set the entry point in settings.CommandServer.module_class_map?".format(modclassname,mod.modobj.__file__))
                try:
                    mod.classinstance = self.start_module(mod.classobj,name)
                except Exception as e:
                    print('Failed to initialize module',name,'due to ',e)
                    traceback.print_exc()
                    print("\n\n")
                    continue
                print("\n\n")
                mod.active = not (mod.classinstance.status != 'idle')
            with self.lock:
                self.modules_dict[name] = mod
        return
        
    def restart_module(self,name):
        """App-accessible call."""
        if module in self.modules_dict:
            mod = self.modules_dict[module]
            mod.active = False
            try:
                mod.classinstance.terminate()
            except Exception as e:
                traceback.print_exc(e)
                print("Problem terminating module",module,", restarting anyway")
        self.start_modules([name])
    
    def deactivate_module(self,name):
        """App-accessible call."""
        if name not in self.modules_dict:
            raise ValueError("Invalid module name")
        self.server['ACTIVITY_STATUS'][name] = 'idle'
        with self.lock:
            mod = self.modules_dict[name]
            if not mod.active:
                return
            mod.active = False
            if mod.apiname is not None:
                #disable in other modules' APIs
                for other in self.modules_dict:
                    if other.jarvis.apis is not None and mod.apiname in other.jarvis.apis:
                        delattr(other.jarvis,mod.apiname)
            if mod.is_managed():
                #it's managed by the CommandServer, deactivate access to other apis
                if mod.jarvis.apis is not None:
                    for apiname in mod.jarvis.apis:
                        delattr(mod.jarvis,apiname)
        
    def activate_module(self,name):
        """App-accessible call."""
        if name not in self.modules_dict:
            raise ValueError("Invalid module name")
        self.server['ACTIVITY_STATUS'][name] = 'active'
        with self.lock:
            mod = self.modules_dict[name]
            if mod.active:
                return
            mod.active = True
            if mod.is_managed():
                #it's managed by the CommandServer, re-activate access to other apis
                if mod.jarvis.apis is not None:
                    for apiname,apihandle in mod.jarvis.apis.items():
                        setattr(mod.jarvis,apiname,apihandle)
            if mod.apiname is not None:
                #re-enable in other modules' APIs
                for other in self.modules_dict:
                    if other.jarvis.apis is not None and mod.apiname in other.jarvis.apis:
                        setattr(other.jarvis,mod.apiname,other.jarvis.apis[mod.apiname])

    def switch_module_activity(self,to_activate,to_deactivate = []):
        """App-accessible call."""
        print('switching module activity:')
        if not to_deactivate:
            if any(s.startswith('App') for s in to_activate):
                print("  auto-deactivating other apps")
                tmp = self.server['ACTIVITY_STATUS'].read()
                for i in tmp.keys():
                    if i.startswith('App') and i not in to_activate:
                        if self.modules_dict[i].active:
                            self.deactivate_module(i)
        else:
            for i in to_deactivate:
                if self.modules_dict[i].active:
                    self.deactivate_module(i)
        for i in to_activate:
            self.activate_module(i)

    def shutdown_all(self):
        self.shutdown()
        print('closing all and exiting')
        for module in self.modules_dict.keys():
            if self.modules_dict[module].classinstance is not None:
                self.modules_dict[module].classinstance.terminate()
            self.modules_dict[module].active = False

    def timeUpdaterLoop(self):
        warning_frequency = ('auto' if self.verbose else 0)
        looper = TimedLooper(trina.settings.get('CommandServer.time_updater_dt'),name='timeUpdater',warning_frequency=warning_frequency)
        while looper:
            if self.shut_down_flag:
                break
            self.server['TRINA_TIME'] = time.time()
    
    def commandReceiverLoop(self):
        command_logger = CommandLogger('EXECUTED_COMMANDS')
        warning_frequency = ('auto' if self.verbose else 0)
        looper = TimedLooper(trina.settings.get('CommandServer.command_receiver_dt'),name='commandReceiver',warning_frequency=warning_frequency)
        trina_queue_reader = jarvis.RedisQueueReader(trina.settings.redis_server_ip())
        while looper:
            if self.shut_down_flag:
                break

            with self.lock:
                module_activities = [(i,mod.active) for (i,mod) in self.modules_dict.items()]
            for i,active in module_activities:
                module_commands = trina_queue_reader.read(str(i)+'_MODULE_COMMANDS')
                #print(i,":",module_commands)
                if module_commands:
                    if active:
                        for command in module_commands:
                            self.runModuleCommand(command)
                                # print(command)
                        for command in module_commands:
                            command_logger.log_command(command,time.time())
                    else:
                        print('ignoring commands from {} because it is inactive'.format(str(i)),module_commands)

    def runModuleCommand(self,command):
        #print("Executing module command",command,"in env",list(self.module_command_env.keys()))
        print("Executing module command",command)
        try:
            exec(command,self.module_command_env)
        except Exception as e:
            print('there was an error executing your command!',e)
            traceback.print_exc()
        finally:
            #print("command received was " + str(command))
            pass

    def moduleMonitorLoop(self):
        heartbeat_tolerance = trina.settings.get('CommandServer.heartbeat_tolerance')
        dt = trina.settings.get('CommandServer.health_monitor_dt')
        warning_frequency = ('auto' if self.verbose else 0)
        looper = TimedLooper(dt,name='moduleMonitor',warning_frequency=warning_frequency)
        while looper:
            if self.shut_down_flag:
                break

            module_activity = self.server['ACTIVITY_STATUS'].read()
            module_health = self.server['HEALTH_LOG'].read()
            #discover other modules run from command line
            for key in module_activity:               
                try:
                    self.modules_dict[key]
                except KeyError as e:
                    if key in module_health:
                        health = module_health[key]
                        if health[0] == 'off' or time.time() - health[1] < heartbeat_tolerance:
                            active = True
                    else:
                        active = True
                    if active:
                        print("\n\n\nDiscovered module",key,"run from command line... not managed by CommandServer\n\n\n")
                        mod = RunningModuleInfo(key,None,None,None)
                        mod.active = (module_activity[key] != 'idle')
                        self.modules_dict[key] = mod
                    else:
                        #clean up keys
                        try:
                            del self.server['HEALTH_LOG'][key]
                        except Exception:
                            pass
                        del self.server['ACTIVITY_STATUS'][key]

            with self.lock:
                managed_restarts = []
                unmanaged_restarts = []
                for key,mod in self.modules_dict.items():
                    if key not in module_health:
                        #assume unmanaged
                        continue
                    health = module_health[key]
                    if health[0] != 'off' and (time.time()-health[1]) > heartbeat_tolerance*dt:
                        if mod.is_managed():
                            print("Module " + key + " is dead  due to timeout, queueing restart")
                            managed_restarts.append(key)
                        else:
                            print("Module " + key + " is dead  due to timeout, removing it from Command Server control")
                            unmanaged_restarts.append(key)
                        self.server["HEALTH_LOG"][key] = [False,time.time()]
                    elif mod.is_managed() and not mod.classinstance.healthy():
                        print("\n\n\nModule " + key + " is dead due to dead process / hung thread, queueing restart\n\n\n")
                        managed_restarts.append(key)

            if managed_restarts:
                print('\n\n\nCommandServer health monitor: restarting modules {}'.format(str(managed_restarts)))
                for module in managed_restarts:
                    try:
                        self.modules_dict[module].classinstance.terminate()
                    except Exception as e:
                        traceback.print_exc(e)
                        print("Problem terminating module",module,", restarting anyway")
                print('\n\n\n')
                self.start_modules(managed_restarts)
            for module in unmanaged_restarts:
                #non-managed, clean this up from the activity / health keys
                with self.lock:
                    del self.modules_dict[module]
                del self.server['ACTIVITY_STATUS'][module]
                try:
                    del self.server['HEALTH_LOG'][module]
                except Exception:
                    pass


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
    parser.add_argument('--modules', default=['Motion','Example','App_DirectTeleOperation'], type=str, nargs='+', help='The list of modules to activate in trina_modules')
    args = parser.parse_args(sys.argv[1:])

    server = CommandServer(modules = args.modules)
    
    while(True):
        time.sleep(100)
        pass
