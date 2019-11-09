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

class PID_Leg_Controller:

    def __init__(self, arduino_port_addr = "/dev/ttyACM0", arduino_baud = 9600, dt = 0.01):
        self.arduino = ArduinoBridge(arduino_port_addr, arduino_baud)
        self.message_header = "TRINA\t"
        self.message_footer = "\tTRINA"
        self.target = 0
        self.current_loc = 0
        self.moving = False
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
            self.sendTargetPositions(0xdead)
            self._updateSensorFeedback()
            if self.current_loc == 0xface:
               self.connected = True
        self.current_loc = None
        self.moving = False

    def sendTargetPositions(self, current_loc):
        self.target = current_loc

        message = self.message_header + str(current_loc) + self.message_footer + "\t"
        cnt = 0
        for char in message:
            if char == '\t':
                cnt+=1
        print(cnt)
        self.arduino.send_message(message)

    def start(self):
        self.sendHandshake()
        time.sleep(0.1)
        print("Connected!")
        self.enabled = True
        self.control_thread.start()

    def stopMotion(self):
        self.sendTargetPositions(self.current_loc)

    def shutdown(self):
        self.enabled = False
        sys.exit(1)

    def getStates(self):
        return self.current_loc, self.moving

    def moving(self):
        return self.moving

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
        if len(vals) != 2:
            return

        try:
            self.current_loc = float(vals[0])
            self.moving = vals[2] != '0'
        except:
            pass

        self.stateRead = False

if __name__ == "__main__":
    t = PID_Leg_Controller()
    t.start()
    time.sleep(1)

    start_time = time.time()
    while(time.time() - start_time < 20):
        t.sendTargetPositions(0.5)
        print(t.getStates())
        time.sleep(0.1)
    print("shutting down...")
    t.shutdown()
