"""A unified way of accessing TRINA settings.

The settings are accessed by calling ``trina_settings.settings()[key1][key2].``
This will produce a dictionary-like object referencing the JSON-distributed
files in the TRINA/Settings folder, starting at TRINA/Settings/root.json.

The other methods, like robot_settings(), robot_codename(), left_arm_dofs(),
etc are all convenience methods.
"""

import json
import os
import weakref
base = os.path.expanduser("~/TRINA/Settings/")
_root = None

class _NestedJSONAccessorFile:
    def __init__(self,fn):
        self.fn = fn
        self.dirty = False
        self.parent = None
        self.children = []
    def save(self,accessor,fn=None):
        val = self.compress(accessor)
        with open(self.fn,'w') as f:
            json.dump(val,f)
        if fn is None:
            if not self.dirty:
                print("NestedJSONAccessor.save(): Not saving JSON file",self.fn,"because it has not been changed")
                return
            for acc in self.children:
                if acc.file.dirty:
                    acc.file.save(acc)
        else:
            for acc in self.children:
                if acc.file.dirty:
                    print("NestedJSONAccessor.save(): Warning, item",acc.mykey,"is dirty but will not be saved since we are saving to a different file than originally opened")
            return
    def compress(self,accessor):
        if accessor.parent is not None and accessor.parent.fn == self.fn:
            if accessor.dirty:
                accessor.parent.val[accessor.key] = accessor.val
            return self.compress(accessor.parent)
        return accessor.val


class _NestedJSONAccessor:
    """Makes it really easy to define a JSON structure distributed across
    files.  With this structure, JSON files can refer to other files using
    values of the form "@FILENAME.JSON" and they will be automatically
    loaded when accessed.

    Almost all dict/list accessors are implemented.  To treat this like a plain
    dict, call asdict().  By default, this will only return the contents of the
    immediate JSON file. To get the whole structure. call asdict(True).
    To treat a list object as a plain list, use aslist() or list(self).

    To save back to disk, call the save() function.  This will preserve the 
    data and file structure as much as possible.
    """
    def __init__(self,val=None,parent=None,key=None):
        self.val=val
        self.fn = None if parent is None else parent.fn
        self.file = None if parent is None else parent.file
        self.mydir = None if parent is None else parent.mydir
        self.valueCache = dict()
        self.mykey = key
        self.parent = parent
        self.dirty = False
    def load(self,fn):
        with open(fn,'r') as f:
            self.val = json.load(f)
        self.fn = fn
        self.file = _NestedJSONAccessorFile(fn)
        self.mydir = os.path.dirname(fn)
    def save(self,fn=None):
        self.file.save(self)
    def __str__(self):
        self._compress()
        return str(self.val)
    def __repr__(self):
        self._compress()
        return repr(self.val)
    def __len__(self):
        return len(self.val)
    def __contains__(self,key):
        return key in self.val
    def __iter__(self):
        self._compress()
        return iter(self.val)
    def keys(self):
        return self.val.keys()
    def values(self):
        raise NotImplementedError("Can't do values() yet")
    def items(self):
        raise NotImplementedError("Can't do items() yet")
    def append(self,v):
        return self.val.append(v)
    def aslist(self):
        if not isinstance(self.val,list):
            raise ValueError("Value is not a list")
        self._compress()
        return self.val
    def asdict(self,full=False):
        if not isinstance(self.val,dict):
            raise ValueError("Value is not a dictionary")
        if full:
            if len(self.valueCache)==0:
                return self.val
            res = self.val.copy()
            for k,v in self.valueCache.items():
                res[k] = v
            return res
        else:
            self._compress(full)
            return self.val
    def _compress(self):
        for k,v in self.valueCache.items():
            if v.file is self.file and v.dirty:
                self.val[k] = v._compress()
        return self.val
    def __setitem__(self,key,value):
        if isinstance(value,_NestedJSONAccessor):
            return self.__setitem__(key,value,val)
        if key in self.valueCache:
            del self.valueCache
        self.val[key] = value
        self.dirty = True
        if self.file is not None:
            self.file.dirty = True
    def __getitem__(self,key):
        if key in self.valueCache:
            return self.valueCache[key]
        try:
            value = self.val[key]
        except KeyError:
            raise KeyError("Invalid key {} for item {} in file {}".format(key,('root' if self.mykey is None else self.mykey),self.fn))
        if isinstance(value,str) and value.startswith('@') and value.endswith('json'):
            res = _NestedJSONAccessor(parent=weakref.proxy(self),key=key)
            fn = value[1:]
            if os.path.isabs(fn):
                res.load(fn)
            else:
                res.load(os.path.join(self.mydir,fn))
            self.file.children.append(res)
            res.file.parent = weakref.proxy(self.file)
            self.valueCache[key] = res
            return res
        else:
            if isinstance(value,(dict,list)):
                #test whether it needs an accessor
                res = _NestedJSONAccessor(value,parent=weakref.proxy(self),key=key)
                self.valueCache[key] = res
                return res
            return value
    def get(self,key,default):
        if key in self.valueCache:
            return self.valueCache[key]
        if not isinstance(self.val,dict):
            raise AttributeError("Can't call get on a NestedJSONAccessor(list), file",self.fn)
        value = self.val.get(key,default)
        if value.startswith('@') and value.endswith('json'):
            res = _NestedJSONAccessor(parent=weakref.proxy(self),key=key)
            fn = value[1:]
            if os.path.isabs(fn):
                res.load(fn)
            else:
                res.load(os.path.join(self.mydir,fn))
            self.file.children.append(res)
            res.file.parent = weakref.proxy(self.file)
            self.valueCache[key] = res
            return res
        else:
            if isinstance(value,(dict,list)):
                res = _NestedJSONAccessor(value,parent=weakref.proxy(self),key=key)
                self.valueCache[key] = res
                return res
            return value

def settings():
    """Returns the global settings object."""
    global _root
    if _root is None:
        fn = os.path.join(base,"root.json")
        _root = _NestedJSONAccessor()
        _root.load(fn)
    return _root

def app_settings(name):
    """Returns the settings for the given app"""
    return settings()[name]

def robot_settings():
    """Returns settings specific to the robot rather than the user or the software system"""
    return settings()['robot']

def user_settings():
    """Returns settings specific to the user rather than the robot or the software system"""
    return settings()['user']

def motion_settings():
    """Returns robot parameter settings specific to the Motion server"""
    return settings()['robot']['motion_server_settings']

def model_settings():
    """Returns robot model settings, e.g., link groups"""
    return settings()['robot']['model_info']

def camera_serial_numbers():
    """Returns dict of camera serial number settings"""
    return settings()["robot"]["camera_serial_numbers"]






def robot_codename():
    """Returns the codename for the robot. """
    return settings()["robot"]["version"]

def robot_model():
    """Returns the URDF model for the robot. """
    return settings()["robot"]["model"]

def robot_model_load():
    """Loads the URDF model for the robot. Returns a WorldModel"""
    from klampt import WorldModel
    w = WorldModel()
    model_path = os.path.join(os.path.expanduser("~/TRINA"),robot_model())
    res = w.readFile(model_path)
    if not res:
        raise RuntimeError("Unable to read robot model "+model_path)
    return w

def robot_home_config():
    """Returns the robot's home configuration."""
    fn = settings()["robot"]["home_config"]
    from klampt.io import loader
    return loader.load('Config',os.path.join(base,fn))

def left_arm_dofs():
    """Returns list of left arm links in model"""
    return list(model_settings()['left_arm_dofs'])

def right_arm_dofs():
    """Returns list of right arm links in model"""
    return list(model_settings()['right_arm_dofs'])

def left_tool_link():
    """Returns left arm tool link in model"""
    return list(model_settings()['left_tool_link'])

def right_tool_link():
    """Returns right arm tool link in model"""
    return list(model_settings()['right_tool_link'])



def motion_server_mode():
    """Returns assumed motion server mode -- note that the motion server
    can actually be started / restarted in a different mode. """
    return settings()["MotionServer"]["mode"]

def motion_server_ip():
    """Returns the motion server IP address """
    return settings()["MotionServer"]["ip_address"]

def motion_server_port():
    """Returns the motion server port"""
    return settings()["MotionServer"]["port"]

def motion_server_addr():
    """Returns formatted address string for the motion server"""
    return "http://%s:%d"%(general()["MotionServer"]["ip_address"],general()["MotionServer"]["port"])

def motion_server_components():
    """Returns the robot components used by the motion server, by default"""
    return list(settings()["MotionServer"]["components"])



def redis_server_ip():
    """Returns the redis server IP address"""
    return general()["RedisServer"]["ip_address"]



def simulation_world_file():
    """Returns a suggested simulation world file"""
    return general()["Simulation"]["world_file"]

def simulation_world_load():
    """Returns a suggested simulation WorldModel"""
    from klampt import WorldModel
    w = WorldModel()
    model_path = os.path.join(os.path.expanduser("~/TRINA"),simulation_world_file())
    res = w.readFile(model_path)
    if not res:
        raise RuntimeError("Unable to read simulation world "+model_path)
    return w


def left_arm_config(name):
    """Returns a named configuration of the robot's left arm. """
    return list(user_settings()["configs"]['left_arm'][name])

def mirror_arm_config(config):
    """Mirrors an arm configuration from left to right or vice versa. """
    RConfig = []
    RConfig.append(-config[0])
    RConfig.append(-config[1]-math.pi)
    RConfig.append(-config[2])
    RConfig.append(-config[3]+math.pi)
    RConfig.append(-config[4])
    RConfig.append(-config[5])
    return RConfig

def right_arm_config(name):
    """Returns a named configuration of the robot's right arm, or a mirrored
    version of the named configuration for the left arm. """
    configs = user_settings()["configs"]
    res = configs['right_arm'].get(name,None)
    if res is None:
        return mirror_arm_config(left_arm_config(name))
    else:
        return list(res)

def add_left_arm_config(name,value,verbose=1):
    """Adds a named configuration of the robot's left arm.  Note: does not
    save to disk"""
    configs = user_settings()["configs"]
    configs['left_arm'][name] = value
    if verbose:
        print("trina_settings: set left_arm config",value,"but have not yet saved")

def add_right_arm_config(name,value,verbose=1):
    """Adds a named configuration of the robot's right arm.  Note: does not
    save to disk"""
    configs = user_settings()["configs"]
    if 'right_arm' not in configs:
        configs['right_arm'] = dict()
    configs['right_arm'][name] = value
    if verbose:
        print("trina_settings: set right_arm config",value,"but have not yet saved")

def save_configs():
    """Saves all the user's changed configs"""
    configs = user_settings()["configs"]
    configs.save()

def edit_left_arm_config(name):
    """Visually edits a named configuration of the robot's left arm.  Note: does not
    save to disk."""
    from klampt import WorldModel,RobotModel
    from klampt.model.subrobot import SubRobotModel
    from klampt.io import resource 
    dofs = left_arm_dofs()
    try:
        q = left_arm_config(name)
    except KeyError:
        q = [0]*len(dofs)
    w = WorldModel()
    model_path = os.path.join(os.path.expanduser("~/TRINA"),robot_model())
    res = w.readFile(model_path)
    if not res:
        raise RuntimeError("Unable to read robot model "+model_path)
    robot = w.robot(0)
    arm_robot = SubRobotModel(robot,dofs)
    save,result = resource.edit(name,q,'Config',world=w,referenceObject=arm_robot)
    if save:
        set_left_arm_config(name,result)

def edit_right_arm_config(name):
    """Visually edits a named configuration of the robot's left arm.  Note: does not
    save to disk."""
    from klampt import WorldModel,RobotModel
    from klampt.model.subrobot import SubRobotModel
    from klampt.io import resource 
    dofs = right_arm_dofs()
    try:
        q = right_arm_config(name)
    except KeyError:
        q = [0]*len(dofs)
    w = WorldModel()
    model_path = os.path.join(os.path.expanduser("~/TRINA"),robot_model())
    res = w.readFile(model_path)
    if not res:
        raise RuntimeError("Unable to read robot model "+model_path)
    robot = w.robot(0)
    arm_robot = SubRobotModel(robot,dofs)
    save,result = resource.edit(name,q,'Config',world=w,referenceObject=arm_robot)
    if save:
        set_right_arm_config(name,result)




def calibration_configs(task):
    """Returns calibration configurations for the given calibration task. """
    fn = settings()['CalibrationModule'][task+'_calibration_configs']
    from klampt.io import loader
    return loader.load('Configs',os.path.join(base,fn))

def edit_calibration_configs(task):
    """Edits calibration configurations for the given calibration task. Saves
    to disk when OK is prssed. """
    fn = settings()['CalibrationModule'][task+'_calibration_configs']
    from klampt.io import loader
    configs = loader.load('Configs',os.path.join(base,fn))
    from klampt import WorldModel,RobotModel
    from klampt.model.subrobot import SubRobotModel
    from klampt.io import resource 
    if sensor == 'right':
        dofs = left_arm_dofs()
    else:
        dofs = right_arm_dofs()
    w = robot_model_load()
    robot = w.robot(0)
    arm_robot = SubRobotModel(robot,dofs)
    #save,result = resource.edit(sensor,configs,'Configs',world=w,referenceObject=arm_robot)
    from klampt.model.trajectory import Trajectory
    save,result = resource.edit(sensor,Trajectory(list(range(len(configs))),configs),'Trajectory',world=w,referenceObject=arm_robot)
    if save:
        print("Saving configs to",os.path.join(base,fn))
        loader.save(result.milestones,'Configs',os.path.join(base,fn))

if __name__ == '__main__':
    #edit_left_arm_config('untucked')
    edit_calibration_configs('left')
