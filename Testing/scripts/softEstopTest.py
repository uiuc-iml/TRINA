import serial
import math
import signal
import sys
import time
from threading import Thread

"""
Class to interface between a python script and an Arduino microcontroller
"""
class ArduinoBridge:
    def __init__(self, port_addr = "/dev/ttyACM0", baud = 115200):
        self.port_addr = port_addr
        self.baud = baud
        self.port = serial.Serial(port_addr, baud,timeout = 0.01)

    def send_message(self, message):
        self.port.write(message)
        self.port.reset_output_buffer()

    # blocks until a message with a "\n" is received
    def read_message(self):
        self.port.reset_input_buffer()
        return self.port.read(size = 1)

class SoftEStop:
    def __init__(self):
        self.arduino = ArduinoBridge()
        self.shutdown_flag = False
        self.control_thread = Thread(target = self._controlLoop)
        self.control_thread.daemon = True
        self.Estopped = True
        self.fault = False
    def start(self):
        self.control_thread.start()
        time.sleep(0.1) #hot fix, if removed, will read estopped for a fraction of a sec, not sure why
        start_time = time.time()
        res = False
        while time.time() - start_time < 3:
            if not self.isEstopped():
                res = True
                break

        if not res:
            self.shutdown()
        
        return res

    def _controlLoop(self):
        while not self.shutdown_flag:
            msg = self.arduino.read_message()
            msg = msg.decode("utf-8")
            # print(msg)
            if msg == '1':
                self.Estopped = False
                self.fault = False
            elif msg == '0':
                self.Estopped = True
                self.fault = False
            else:
                self.Estopped = True
                self.fault = True
            
            time.sleep(0.005)

    def shutdown(self):
        self.shutdown_flag = True
        return 

    def isFault(self):
        return self.fault

    def isEstopped(self):
        return self.Estopped

if __name__ == "__main__":
    estop = SoftEStop()
    res = estop.start()
    print('-------------------')
    if res:
        for i in range(5000):
            print(estop.isEstopped())
            time.sleep(0.01)
    else:
        print('failure')
    estop.shutdown()