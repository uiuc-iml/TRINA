from klampt import WorldModel,Simulator
from .import settings
import os
import logging
from datetime import datetime

def trina_root():
    return os.path.expanduser("~/TRINA")

def models_root():
    return os.path.join(trina_root(),"Models")

def robot_models_root():
    return os.path.join(trina_root(),"Models/robots")

def log_root():
    return os.path.join(trina_root(),"errorLogs")

def robot_model_load():
    """Loads the URDF model for the robot. Returns a WorldModel"""
    w = WorldModel()
    model_path = os.path.join(trina_root(),settings.robot_model())
    res = w.readFile(model_path)
    if not res:
        raise RuntimeError("Unable to read robot model "+model_path)
    return w

def simulation_world_load():
    """Returns a suggested simulation WorldModel"""
    from klampt import WorldModel
    w = WorldModel()
    model_path = os.path.join(trina_root(),settings.simulation_world_file())
    res = w.readFile(model_path)
    if not res:
        raise RuntimeError("Unable to read simulation world "+model_path)
    return w

def set_robot_sensor_calibration(world):
    """Returns a Simulator that is calibrated according to the 
    sensor calibration in Settings/"""
    sim = Simulator(world)
    robot = sim.controller(0)
    all_cams = []
    i = 0
    while True:
        sensori = robot.sensor(i)
        if sensori.type() == 'CameraSensor':
            all_cams.append(sensori.name())
        if sensori.type() == '':  #out of sensors
            break
        i += 1

    from klampt.model import sensing
    from klampt.math import se3
    from klampt.io import loader
    camera_settings = settings.camera_settings()
    for camera in camera_settings:
        cam = robot.sensor(camera)
        if len(cam.name()) > 0: # valid camera
            print("Updating simulated camera",cam.name(),"to match calibration in TRINA/Settings/")
            calib_fn = camera_settings[camera]["transform"]
            calib_fn = os.path.join(trina_root(),calib_fn)
            if calib_fn.endswith('npy'):
                import numpy as np    
                transform = np.load(open(calib_fn, 'rb'))
                sensing.set_sensor_xform(cam,se3.from_homogeneous(transform))
            else:
                transform = loader.load(calib_fn)
                sensing.set_sensor_xform(cam,transform)
    return sim


def get_logger(name, level=None, filename=None):
    if not os.path.exists(log_root()):
        os.makedirs(log_root())
    logger = logging.getLogger(name)
    if filename == None:
        filename = log_root()+"/logFile_" + datetime.now().strftime('%d%m%Y') + ".log"
    if level == None:
        level = logging.DEBUG
    logFormatter = logging.Formatter("%(asctime)s [%(levelname)s] %(process)s (%(threadName)-9s) %(name)s: %(message)s")
    logger = logging.getLogger(name)
    fileHandler = logging.FileHandler(filename)
    consoleHandler = logging.StreamHandler()
    fileHandler.setFormatter(logFormatter)
    logger.addHandler(fileHandler)
    consoleHandler.setFormatter(logFormatter)
    logger.addHandler(consoleHandler)
    logger.setLevel(level)
    return logger
