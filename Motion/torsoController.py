import serial
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
        self.port = serial.Serial(port_addr, baud)

    def send_message(self, message):
        self.port.reset_output_buffer()
        self.port.write(message)

    # blocks until a message with a "\n" is received
    def read_message(self):
        self.port.reset_input_buffer()
        return self.port.readline()

class TorsoController:

    def __init__(self, arduino_port_addr = "/dev/ttyACM0", arduino_baud = 115200, dt = 0.025):
        self.arduino = ArduinoBridge(arduino_port_addr, arduino_baud)
        self.message_header = "T\t"
        self.message_footer = "\tT"

        self.target_tilt = 0
        self.target_height = 0
        self.target_leg = 0

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
            message = self.message_header + str(0xdead) + "\t" + str(0xbeef) + "\t" + str(0) + self.message_footer + "\t"
            self.arduino.send_message(message)

            rv = self._updateSensorFeedback()
            if rv and self.height == 0xface and self.tilt == 0xb00c:
                self.connected = True

            time.sleep(self.dt)

        self.height = None
        self.tilt = None
        self.lift_moving = False
        self.tilt_moving = False

    def sendTargetPositions(self, height, tilt, leg):
        self.target_height = height
        self.target_tilt = tilt
        self.target_leg = leg

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
            rv = self._updateSensorFeedback()
            if not rv:
                time.sleep(self.dt)
                continue
            
            message = self.message_header + str(self.target_height) + "\t" + str(self.target_tilt) + "\t" + str(self.target_leg) + self.message_footer + "\t"
            self.arduino.send_message(message)

            time.sleep(self.dt)

    def _updateSensorFeedback(self):
        message = self.arduino.read_message()
        """
        if "bad" in message:
            print("bad")
        else:
            print("good")
        """

        header_idx = message.find(self.message_header)
        footer_idx = message.find(self.message_footer)

        if header_idx == -1 or footer_idx == -1:
            return

        parsed_message = message[header_idx+len(self.message_header) + 1 : footer_idx]
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
    t = TorsoController(arduino_port_addr = "/dev/ttyACM0", dt = 0.001)
    t.start()

    start_time = time.time()
    i=0

    t.sendTargetPositions(0.29, 39, 1.0)

    while(time.time() - start_time < 20):
        print(i, t.getStates())
        time.sleep(0.01)
        i += 1

    print("shutting down...")
    t.shutdown()
