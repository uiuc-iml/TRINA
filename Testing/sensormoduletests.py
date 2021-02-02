from sensorModule import Camera_Robot
import time
from matplotlib import pyplot as plt
a = Camera_Robot()
time.sleep(1)
b = []

for i in range(50):
    b = a.get_point_clouds()
    print(b.keys())
    print(b[b.keys()[0]])
    time.sleep(0.08)
    if(int(time.time())%2 == 0):
        plt.close('all')