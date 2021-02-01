import time
import sys
import time
from .api import APILayer
import redis

class Jarvis(APILayer):
    """Currently implemented APIs are command_server (top level APIs), motion, and sensors.

    Usage::

        jarvis = [Jarvis instance, set up for you by command_server]
        components = jarvis.robot.components()
        mode = jarvis.robot.mode()
        jarvis.ui.getRayClick()
        etc...
    
    """
    def __init__(self, moduleName, apis=None, server=None):
        if server is None:
            #initialize
            from ..state_server import StateServer
            server = StateServer()
            server.set(['HEALTH_LOG',moduleName],['off',0])
            server.set(['ACTIVITY_STATUS',moduleName],'active')
            #initialize basic APIs available to the module
        
        APILayer.__init__(self,moduleName,server)
        self.server = server
        self.moduleName = moduleName
        self.apis = apis
        if isinstance(apis,(str,list)):
            self.apis = dict()
            self.require(apis)
        elif apis is not None:
            assert isinstance(apis,dict),"apis must be a dict mapping api names to handles"
            for apiname,apihandle in apis.items():
                assert apiname not in self.__dict__,"Can't define a module called "+apiname+" because it conflicts with an Jarvis attribute"
                if apiname != apihandle.name():
                    print("trina.jarvis.Jarvis: warning, API",apihandle.name(),"is being stored under different identifier,",apiname)
                setattr(self,apiname,apihandle)
        else:
            self.apis = dict()

    @classmethod
    def name(cls):
        return 'jarvis'

    def require(self,modules):
        """Declares that a certain set of APIs are required by this module.
        
        Returns the list of api names provided by these modules, or None 
        if the corresponding module could not be found.
        """
        extract = False
        if isinstance(modules,str):
            modules = [modules]
            extract = True
        apimap = get_api_classes(modules)
        result = []
        for mod,apicls in apimap.items():
            apiname = apicls.name()
            if apiname in self.apis:
                result.append(apiname)
                continue
            if apicls.interprocess():
                try:
                    self.apis[apiname] = apicls(self.moduleName,self.server)
                    setattr(self,apiname,self.apis[apiname])
                    result.append(apiname)
                except Exception as e:
                    print("trina.jarvis.Jarvis: warning, API",apicls.name(),"from module",mod,"could not be constructed with default arguments, skipping")
                    import traceback
                    traceback.print_exc()
                    result.append(None)
            else:
                print("trina.jarvis.Jarvis: warning, requested API",apicls.name(),"from module",mod,"does not support interprocess access, skipping")
                import traceback
                traceback.print_exc()
                result.append(None)
        if extract:
            return result[0]
        return result

    def log_health(self, status=True):
        """Add the health log of the module
        """
        self._redisSet(["HEALTH_LOG",self.moduleName],[status, time.time()])

    def enableHealthChecks(self, status=True):
        """If you are using a dummy module, should disable health checks."""
        self._redisSet(["HEALTH_LOG",self.moduleName],[True if status else 'off', time.time()])

    def getActivityStatus(self):
        """Returns the module activity status, as controlled by Command Server.
        The module should respect a request to switch to 'idle' by stopping 
        internal processes promptly.

        Returns:
            dict
        """
        return self._redisGet(['ACTIVITY_STATUS',self.moduleName])

    def switchModuleActivity(self, to_activate, to_deactivate=[]):
        """Changes the status of one or more modules
        """
        self._moduleCommand('switch_module_activity', to_activate, to_deactivate)

    def restartModule(self, otherModule):
        self._moduleCommand('restart_module', otherModule)

    def activateModule(self,module):
        self._moduleCommand('activate_module', module)

    def deactivateModule(self,module):
        self._moduleCommand('deactivate_module', module)

    def getTrinaTime(self):
        """Returns the trina time
        """
        try:
            return self._redisGet(['TRINA_TIME'])
        except Exception:
            return None

    def getUIState(self):
        """ Return UI state dictionary

        Returns:
            dict
        """
        try:
            return self._redisGet(["UI_STATE"])
        except Exception:
            return None

    def getRobotState(self):
        """ Return robot state dictionary

        Returns:
            dict
        """
        try:
            return self._redisGet(["ROBOT_STATE"])
        except Exception:
            return None

    def getSimulatedWorld(self):
        """ Return the simulated world

        Returns:
            str: The Klampt world of the simulated robot.
        """
        try:
            return self._redisGet(["SIM_WORLD"])
        except Exception:
            return None


def get_module_classes(names,doreload=True):
    """Returns a dict mapping module names to Module classes, for the modules
    named in names.  If doreload=True, the Python modules will be reloaded to
    reflect any changes made since the Python kernel started (default).
    """
    import trina
    from importlib import import_module
    if sys.version_info[0] >= 3:
        from importlib import reload
    module_locations = trina.settings.app_settings('Jarvis')['module_locations']
    app_locations = trina.settings.app_settings('Jarvis')['app_locations']
    mod_class_map = trina.settings.app_settings('Jarvis')['module_class_map']
    res = dict()
    for name in names:
        briefName = name  #this is the name used by the Python module
        if name.startswith('App_'):
            briefName = name[4:]
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
        if doreload:
            reload(modobj)
            for k in list(sys.modules):
                if k.startswith(modobj.__name__):
                    reload(sys.modules[k])
        modclassname = mod_class_map.get(name,name)
        try:
            modclass = getattr(modobj,modclassname)
        except AttributeError:
            raise RuntimeError("Class {} not found in module {}; perhaps you should set the entry point in settings.Jarvis.module_class_map?".format(modclassname,modobj.__file__))
        res[name] = modclass
    return res


def get_api_classes(names,doreload=True):
    """Returns a dict whose keys are modules and values are APILayer classes,
    for the modules named in names.  If doreload=True, the Python modules will
    be reloaded to reflect any changes made since the Python kernel started
    (default).
    """
    import trina
    from importlib import import_module
    if sys.version_info[0] >= 3:
        from importlib import reload
    module_locations = trina.settings.app_settings('Jarvis')['module_locations']
    mod_class_map = trina.settings.app_settings('Jarvis')['module_class_map']
    res = dict()
    for name in names:
        briefName = name  #this is the name used by the Python module
        if name.startswith('App_'):
            print('trina.jarvis.get_api_classes: requested an API for an App class?')
            continue
        modobj = None
        for location in module_locations:
            try:
                modobj = import_module(location+'.'+briefName)
            except ImportError:
                pass
        if modobj is None:
            raise RuntimeError("Invalid module name {}, could not be found/loaded in {}".format(briefName,module_locations))
        #for safety
        if doreload:
            reload(modobj)
            for k in list(sys.modules):
                if k.startswith(modobj.__name__):
                    reload(sys.modules[k])
        modclassname = mod_class_map.get(name,name)
        try:
            modclass = getattr(modobj,modclassname)
        except AttributeError:
            raise RuntimeError("Class {} not found in module {}; perhaps you should set the entry point in settings.Jarvis.module_class_map?".format(modclassname,modobj.__file__))
        try:
            apiclass = modclass.apiClass()
        except AttributeError:
            print('trina.jarvis.get_api_classes: Module {}, file {} does not have apiClass method, skipping'.format(modclassname,modclass.__file__))
            continue
        res[name] = apiclass
    return res


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
