from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import time
import math


interface = RedisInterface(host="localhost")
interface.initialize()
server = KeyValueStore(interface)
side = 'right'

server['UI_STATE']["controllerButtonState"][side+'Controller']["press"] = [True, False, False, False]
print("Sending home signal")
time.sleep(1)
server['UI_STATE']["controllerButtonState"][side+'Controller']["press"] = [False, False, False, False]
print("Waiting for 10s")
time.sleep(10)
print("Set init controller transform")
server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerPosition'] = [0,0,0]
server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerRotation'] = [1,0,0,0,1,0,0,0,1]
server['UI_STATE']["controllerButtonState"][side+'Controller']["press"] = [False, True, False, False]
time.sleep(1)
server['UI_STATE']["controllerButtonState"][side+'Controller']["press"] = [False, False, False, False]

print("Move in sinusoidal motion")
server['UI_STATE']["controllerButtonState"][side+'Controller']['squeeze'] = [0, 1]
dt = 0.02
t = 0

while True:
    server['UI_STATE']["controllerPositionState"][side+'Controller']['controllerPosition'] = [0,0,0.1 * math.sin(t*dt)]
    t += 1
    time.sleep(dt)
