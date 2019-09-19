import CRC16
import serial
import ur5_config as UR5_CONSTANTS
import sys

def addCRC(myHex):
    #takes a hex string and adds a modbus CRC to it
    crc = CRC16.calcString(myHex, CRC16.INITIAL_MODBUS)
    flipped_res = "{0:#0{1}x}".format(crc, 6)
    res = flipped_res[4:6] + flipped_res[2:4]
    add_on = res.decode('hex')
    newHex = myHex + add_on
    return newHex

class Robotiq2Controller:
    def __init__(self,port='/dev/ttyUSB0'):
        self.port = port
        self._gripper_ser = None       
        #1 is open, 0 is closed

        #keeps track of the last read state of the gripper in case of gripper failure
        #happened occasionally was hard to determine cause
        self._last_gripper = 0
        #openGripper is 00
        #closeGripper is FF

    def start(self):
        try:
            self._gripper_ser = serial.Serial(port=self.port,baudrate=115200,timeout=1,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)

            #sends a gripper command to initiate the gripper
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
            data_raw = self._gripper_ser.readline()
            time.sleep(0.1)
            #sends a command to check the status
            self._gripper_ser.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
            data_raw =self._gripper_ser.readline()
            time.sleep(1)
            # This command makes the gripper start in the open configuration on startup 
            self._gripper_ser.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")
            data_raw = self._gripper_ser.readline()
            time.sleep(1)
            
            self._gripper_ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
            data_raw = self._gripper_ser.readline()
            time.sleep(0.1)
        except:
            print(sys.exc_info()[0])
            raise RuntimeError("Warning, gripper not set up")

    def command(self, q=[], qd=[], delay_t=UR5_CONSTANTS.DEFAULT_GRIPPER_DELAY):
        if sum([ x is not None for x in [q, qd]]) == 1:
            if(not self._gripper_ser):
                print("Error, gripper not enabled")
                return        
            #we are doing either q or qd
            #            0   1   2   3   4   5   6   7   8   9  10  11  12
            command = "\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29"
            # default command for gripper                 position^   ^speed
            #            write to 3 registers^   ^6 bytes total     force^    ^CRC
            #00 is min speed, also min position (open)
            #FF is max speed, also max position (close)
            if q:
                if len(q) == 1:
                    gripper_pos = q[0]
                    if gripper_pos <= 1 and gripper_pos >= 0:
                        gripper_command = chr(int(gripper_pos*255))
                        gripper_vel_command = "\xFF"
                        #if position is set, we set command to maximum velocity
                        command = command[0:10] + gripper_command + gripper_vel_command + command[12]
            elif qd:
                if len(qd) == 1:
                    gripper_vel = qd[0]
                    if abs(gripper_vel) <= 1 and abs(gripper_vel) > 0:
                        if(gripper_vel < 0):
                            #if vel negative, open (go towards 0)
                            gripper_vel_command = chr(int(abs(gripper_vel)*255))
                            gripper_command = chr(int(00))
                            command = command[0:10] + gripper_command + gripper_vel_command + command[12]
                        else:
                            #if vel positive, close (go towards 1)
                            gripper_vel_command = chr(int(abs(gripper_vel)*255))
                            gripper_command = chr(int(255))
                            command = command[0:10] + gripper_command + gripper_vel_command + command[12]
                    elif gripper_vel == 0:
                        #if vel is 0, stop gripper
                        gripper_vel_command = chr(int(00))
                        current_position = self._readGripper()
                        gripper_command = chr(int(current_position*255))
                        command = command[0:10] + gripper_command + gripper_vel_command + command[12]
                        #set gripper to "0"
                        #really set gripper to minspeed and stop moving
            else:
                #can return without setting gripper
                #if we get here, something has gone wrong
                print "Warning, neither q nor dq set for gripper"
                return
            #command for adding CRC is at the top of file
            #takes full command, appends CRC and returns appended command
            #Gripper needs CRC for error checking
            command = addCRC(command)
            self._gripper_ser.write(command)
            data = self._gripper_ser.read(8)

    #students don't use these functions. They interface with the robot through setConfig and setVelocity
    def read(self):
        #returns float 0-1
        #0 is open, 1 is closed
        if(not self._gripper_ser):
            print("Error, gripper not enabled")
            return 0
        #self._gripper_ser.flush()
        self._gripper_ser.write("\x09\x03\x07\xD0\x00\x03\x04\x0E")
        #this says read three registers starting from 07D0
        #response should be like:
        #09 03 06 E000 0000 0000 0000
        #         dat1 dat2 dat3 CRC
        #dat1 = Gripper Status, Object Detection, 
        #dat2 = Fault Status and Position Request Echo
        #dat3 = Position, Current
        data_raw = self._gripper_ser.read(size=11)

        data = binascii.hexlify(data_raw)
        position = data[14:16]
        float_position = self._last_gripper
        try:
            int_position = int(position, 16)
            float_position = int_position/255.0
            self._last_gripper = float_position
            #sometimes, the gripper has trouble reading quickly
        except:
            print "gripper read exception"
        return float_position
