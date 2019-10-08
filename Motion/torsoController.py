import serial
import time

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

    def __init__(self, arduino_port_addr = "/dev/ttyACM0", arduino_baud = 9600):
        self.arduino = ArduinoBridge(arduino_port_addr, arduino_baud)
        self.message_header = "TRINA\t"
        self.message_footer = "\tTRINA"

    def send_target_positions(self, height, tilt):
        message = self.message_header + "\0" + str(height) + "\0" + str(tilt) + "\0" + self.message_header
        self.arduino.send_message(message)

    def get_positions(self):
        message = self.arduino.read_message()
        header_idx = message.find(self.message_header)
        footer_idx = message.find(self.message_footer)

        if header_idx == -1 or footer_idx == -1:
            print("invalid serial data!")
            return

        parsed_message = message[header_idx+6 : footer_idx-1]
        vals = parsed_message.split("\t")
        if len(vals) != 2:
            print("invalid serial data!")
            return

        tilt = float(vals[0])
        height = float(vals[1])
        return tilt, height

if __name__ == "__main__":
    t = TorsoController()
    #t.send_target_positions(10.0, 12.0)
    while True:
        print(t.get_positions())
        time.sleep(0.1)
