import serial
import signal
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
        self.height = 0
        self.tilt = 0
        self.tilt_limit_switch = False
        self.lift_limit_switch = False
        self.control_thread = Thread(target = self._controlLoop)
        self.enabled = False
        self.dt = dt
        self.connected = False
        # calls shutdown() upon catching SIGINT
        signal.signal(signal.SIGINT, self._sigintHandler)
        self.sendHandshake()

    def sendHandshake(self):
        self.arduino.send_message("TRINA_CONNECTION_CHECK")
        while not self.connected:
            if self.arduino.read_message() != "TRINA_CONNECTION_CHECK":
                time.sleep(self.dt)
            else:
                self.connected = True

    def sendTargetPositions(self, height, tilt):
        message = str(height) + "\0" + str(tilt) + "\0"
        print(message)
        self.arduino.send_message(message) 

    def start(self):
        if not self.connected:
            return
        self.enabled = True
        self.control_thread.start()

    def stopMotion(self):
        self.sendTargetPositions(self.height, self.tilt)

    def shutdown(self):
        self.enabled = False

    def getStates(self):
        return self.tilt, self.height, self.tilt_limit_switch, self.lift_limit_switch

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
            print("invalid serial data!")
            return

        parsed_message = message[header_idx+6 : footer_idx]
        vals = parsed_message.split("\t")
        if len(vals) != 4:
            print("invalid serial data!")
            return

        try:
            self.tilt = float(vals[0])
            self.height = float(vals[1])
            self.tilt_limit_switch = vals[2] != '0'
            self.lift_limit_switch = vals[3] != '0'
        except:
            print("invalid serial data!") 

if __name__ == "__main__":
    t = TorsoController()
    print("starting...")
    t.start()

    start_time = time.time()
    while(time.time() - start_time < 5):
        time.sleep(0.25)
    print("setting target!")
    t.sendTargetPositions(0.32, 0)

    start_time = time.time()
    while(time.time() - start_time < 20):
        print(t.getStates())
        time.sleep(0.25)
    print("shutting down...")
    t.shutdown()
