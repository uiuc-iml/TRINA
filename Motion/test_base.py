from baseController import BaseController
import time
import math

import sys
import signal

def sigint_handler(signum, frame):
    assert(signum == signal.SIGINT)
    print("SIGINT caught...shutting down!")
    if base:
        base.shutdown()

signal.signal(signal.SIGINT, sigint_handler) # catch ctrl-c

dt = 0.01
start_time = time.time()
TIME = 10
base = BaseController(dt)
print('starting')
base.start()
print('started')

start_time = time.time()
target = 1
ramp_time = 5

while time.time() - start_time < ramp_time/2:
    base.setCommandedVelocityRamped([target, target], ramp_time)
    print(base.getMeasuredVelocity())
    time.sleep(0.01)

print("break")

while time.time() - start_time < ramp_time:
    base.setCommandedVelocity([target, target])
    print(base.getMeasuredVelocity())
    time.sleep(0.01)

print('done')
base.shutdown()
