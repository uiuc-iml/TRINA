"""A unified way of accessing TRINA settings.

The settings are accessed by calling ``trina.settings.get(key1.key2)`` or
``trina.settings.settings()[key1][key2]``.

This will produce a dictionary-like object referencing the JSON-distributed
files in the TRINA/Settings folder, starting at TRINA/Settings/root.json.

The other methods, like :func:`robot_settings`, :func:`robot_codename`,
:func:`left_arm_dofs`, etc are all convenience methods.
"""

import json
import os
import weakref
from .utils import NestedJsonAccessor
base = os.path.expanduser("~/TRINA/Settings/")
_root = None

def settings():
    """Returns the global settings object."""
    global _root
    if _root is None:
        fn = os.path.join(base,"root.json")
        _root = NestedJsonAccessor()
        _root.load(fn)
    return _root

def reload():
    """Reloads everything from files. """
    global _root
    fn = os.path.join(base,"root.json")
    _root = NestedJsonAccessor()
    _root.load(fn)
    return _root

def get(path,decode=True):
    """Retrieves a setting from a period-separated string.  If decode=True,
    this decodes the NestedJsonAccessor object."""
    items = path.split('.')
    res = settings()
    for key in items:
        res = res[key]
    if decode and isinstance(res,NestedJsonAccessor):
        try:
            return res.asdict()
        except:
            return res.aslist()
    return res

def set(path,value,dosave=False):
    """Sets a setting to some value from a period-separated string.
    If dosave == True, then the settings are saved"""
    items = path.split('.')
    if len(items) == 0:
        raise "Can't set an empty string"
    res = settings()
    lastAccessor = res
    for key in items[:-1]:
        res = res[key]
        if isinstance(res,NestedJsonAccessor):
            lastAccessor = res
    if dosave:
        lastAccessor.save()

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
    """Returns robot-specific parameter settings being used by the Motion server"""
    return settings()['robot']['motion_server_settings']

def model_settings():
    """Returns robot model settings, e.g., link groups"""
    return settings()['robot']['model_info']

def camera_settings():
    """Returns camera settings, e.g., serial numbers, links, transforms"""
    return settings()["robot"]["camera_settings"].asdict()






def robot_codename():
    """Returns the codename for the robot. """
    return settings()["robot"]["version"]

def robot_model():
    """Returns the URDF model for the robot. """
    return settings()["robot"]["model"]

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
    return model_settings()['left_tool_link']

def right_tool_link():
    """Returns right arm tool link in model"""
    return model_settings()['right_tool_link']



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
    mserver = settings()["MotionServer"]
    return "http://%s:%d"%(mserver["ip_address"],mserver["port"])

def motion_server_components():
    """Returns the robot components used by the motion server, by default"""
    return list(settings()["MotionServer"]["components"])



def redis_server_ip():
    """Returns the redis server IP address"""
    return settings()["RedisServer"]["ip_address"]



def simulation_world_file():
    """Returns a suggested simulation world file"""
    return settings()["Simulation"]["world_file"]



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
