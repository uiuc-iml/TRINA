import serial
import signal
import sys
import time
from threading import Thread

"""
Class to interface between a python script and an Arduino microcontroller on Support Leg
"""
class ArduinoBridge:

    def __init__(self, port_addr = "/dev/ttyACM0", baud = 9600):
        self.port_addr = port_addr
        self.baud = baud
        self.port = serial.Serial(port_addr, baud)

    def send_message(self, message):
        self.port.write(message)

    # blocks until a message with a "\n" is received
    def read_message(self):
        return self.port.readline()

        
    class SupportLegController():

         def __init__(self, arduino_port_addr = "/dev/ttyACM0", arduino_baud = 9600, dt = 0.01):
            self.arduino = ArduinoBridge(arduino_port_addr, arduino_baud)
            self.message_header = "TRINA\t"
            self.message_footer = "\tTRINA"
            self.target_tilt = 0
            self.target_height = 0
            self.current_position = 0
            self.is_moving = False
            self.control_thread = Thread(target = self._controlLoop)
            self.enabled = False
            self.dt = dt
            self.stateRead = False
            self.connected = False
            # calls shutdown() upon catching SIGINT
            signal.signal(signal.SIGINT, self._sigintHandler) 

         def sendHandshake(self):
            # wait for the arduino's message
            while not self.connected:
                self.sendTargetPositions(0xdead, 0xbeef)
                self._updateSensorFeedback()
                if self.height == 0xface and self.tilt == 0xb00c:
                    self.connected = True
            self.height = None
            self.tilt = None
            self.lift_moving = False
            self.tilt_moving = False