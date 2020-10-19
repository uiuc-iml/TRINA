from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
import time
import math


interface = RedisInterface(host="localhost")
interface.initialize()
server = KeyValueStore(interface)

print(server['UI_STATE']["controllerPositionState"].read())
server['UI_STATE']["controllerButtonState"]['leftController']["press"] = [True, False, False, False]
print("Sending home signal")
time.sleep(1)
server['UI_STATE']["controllerButtonState"]['leftController']["press"] = [False, False, False, False]
print("Waiting for 10s")
time.sleep(10)
print("Set init controller transform")
server['UI_STATE']["controllerPositionState"]['leftController']['controllerPosition'] = [0,0,0]
server['UI_STATE']["controllerPositionState"]['leftController']['controllerRotation'] = [1,0,0,0,1,0,0,0,1]
server['UI_STATE']["controllerButtonState"]['leftController']["press"] = [False, True, False, False]
time.sleep(1)
server['UI_STATE']["controllerButtonState"]['leftController']["press"] = [False, False, False, False]
print(server['UI_STATE']["controllerPositionState"].read())

print("Move in sinusoidal motion")
server['UI_STATE']["controllerButtonState"]['leftController']['squeeze'] = [0, 1]
dt = 0.02
t = 0

while True:
    server['UI_STATE']["controllerPositionState"]['leftController']['controllerPosition'] = [0,0,0.1 * math.sin(t*dt)]
    # print(server['UI_STATE']["controllerPositionState"]['leftController']['controllerPosition'].read())
    t += 1
    time.sleep(dt)
