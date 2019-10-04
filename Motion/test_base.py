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

base.setPath([(0, 0, 0), (2, 2, math.pi/2), (5, 5, 0)], 0.5)
while not base.isPathDone():
    if base.newState():
	print(base.getPosition())
	base.markRead()
    time.sleep(dt)

print('done')
base.shutdown()
