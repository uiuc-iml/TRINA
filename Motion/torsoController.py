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
        signal.signal(signal.SIGINT, self._sigintHandler)

    def sendTargetPositions(self, height, tilt):
        message = str(height) + "\0" + str(tilt) + "\0"
        print(message)
        self.arduino.send_message(message)

    def _sigintHandler(self, signum, msg):
        assert(signum == signal.SIGINT)
        self.shutdown()

    def _controlLoop(self):
        while self.enabled:
            self._updateSensorFeedback()
            time.sleep(self.dt)

    def start(self):
        self.enabled = True
        self.control_thread.start()

    def shutdown(self):
        self.enabled = False

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

    def get_positions(self):
        return self.tilt, self.height, self.tilt_limit_switch, self.lift_limit_switch

if __name__ == "__main__":
    t = TorsoController()
    print("starting...")
    t.start()
    start_time = time.time()
    while(time.time() - start_time < 5):
        t.sendTargetPositions(time.time() - start_time, 100)
        print(t.get_positions())
        time.sleep(0.5)
    print("shutting down...")
    t.shutdown()
