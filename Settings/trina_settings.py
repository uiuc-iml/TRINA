import json
import os
base = os.path.expanduser("~/TRINA/Settings/")
_settings = dict()

def settings(name):
    global _settings
    if name in _settings:
        return _settings[name]

    fn = os.path.join(base,name+".json")
    with open(fn,'r') as f:
        _settings[name] = json.load(f)
    return _settings[name]

def general():
    return settings('general')

def arm_links():
    return settings('arm_links')

def calibrations():
    return settings('calibrations')

def left_arm_common():
    return settings('left_arm_common')

def ip():
    return settings('ip')

def camera_serial_numbers():
    return settings('camera_serial_numbers')






def robot_codename():
    return general()["version"]

def robot_model():
    return general()["model"]

def motion_server_mode():
    return general()["MotionServer"]["mode"]

def motion_server_ip():
    return "http://%s:%d"%(general()["MotionServer"]["ip_address"],general()["MotionServer"]["port"])