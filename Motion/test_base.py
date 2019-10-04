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

while time.time() - start_time < TIME:
    base.setCommandedVelocity((0, 0.5))
    if base.newState():
	print(base.getPosition())
	base.markRead()
    time.sleep(dt)
base.shutdown()
