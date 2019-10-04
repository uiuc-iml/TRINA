from BaseController import BaseController
import time
import math

dt = 0.01
start_time = time.time()
TIME = 5
base = BaseController(dt)
base.start()

while time.time() - start_time < TIME:
    base.setVelocity(0, 0.01)
    time.sleep(0.01)
base.shutdown()
