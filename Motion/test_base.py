from BaseController import BaseController
import time
import math

dt = 0.01
base = BaseController(dt)
base.start()

base.setPath([(0, 0, 0), (2, 2, math.pi/2), (5, 5, 0)], 0.5)
while not base.isPathDone():
    time.sleep(0.01)
base.shutdown()
print("Done!")
