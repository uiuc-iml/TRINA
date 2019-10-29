import serial
import signal
import sys
import time
from threading import Thread

"""
Class to interface between a python script and an Arduino microcontroller
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

class TorsoController:

    def __init__(self, arduino_port_addr = "/dev/ttyACM0", arduino_baud = 9600, dt = 0.01):
        self.arduino = ArduinoBridge(arduino_port_addr, arduino_baud)
        self.message_header = "TRINA\t"
        self.message_footer = "\tTRINA"
        self.target_tilt = 0
        self.target_height = 0
        self.height = 0
        self.tilt = 0
        self.lift_moving = False
        self.tilt_moving = False
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

    def sendTargetPositions(self, height, tilt):
        self.target_height = height
        self.target_tilt = tilt

        message = self.message_header + str(height) + "\t" + str(tilt) + self.message_footer + "\t"
        self.arduino.send_message(message) 

    def start(self):
        self.sendHandshake()
        time.sleep(0.1)
        print("Connected!")
        self.enabled = True
        self.control_thread.start()

    def stopMotion(self):
        self.sendTargetPositions(self.height, self.tilt)

    def shutdown(self):
        self.enabled = False
        sys.exit(1)

    def getStates(self):
        return self.tilt, self.height, self.tilt_moving, self.lift_moving

    def moving(self):
        return self.lift_moving or self.tilt_moving

    def newState(self):
        return not self.stateRead

    def markRead(self):
        self.stateRead = True

    def _sigintHandler(self, signum, msg):
        assert(signum == signal.SIGINT)
        self.shutdown()

    def _controlLoop(self):
        while self.enabled:
            self._updateSensorFeedback()
            time.sleep(self.dt)

    def _updateSensorFeedback(self):
        message = self.arduino.read_message()

        header_idx = message.find(self.message_header)
        footer_idx = message.find(self.message_footer)

        if header_idx == -1 or footer_idx == -1:
            return

        parsed_message = message[header_idx+6 : footer_idx]
        vals = parsed_message.split("\t")
        if len(vals) != 4:
            return

        try:
            self.tilt = float(vals[0])
            self.height = float(vals[1])
            self.tilt_moving = vals[2] != '0'
            self.lift_moving = vals[3] != '0'
        except:
            pass

        self.stateRead = False

if __name__ == "__main__":
    t = TorsoController()
    t.start()

    t.sendTargetPositions(0.35, 0)

    start_time = time.time()
    while(time.time() - start_time < 20):
        print(t.getStates())
        time.sleep(0.1)
    print("shutting down...")
    t.shutdown()
