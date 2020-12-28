import serial
import os
import math
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import time
from threading import Thread, Lock, RLock
import threading
from copy import copy
DEGREE_2_RADIAN = 2.0*math.pi/180.0

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_tilt                 = 1                 # Dynamixel ID : 1, FOR PITCH 
DXL_ID_pan                  = 2                 # Dynamixel ID : 2, FOR YAW
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
# DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
# DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
# DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


class HeadController:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.dynamixel = PacketHandler(PROTOCOL_VERSION)
        self.active = False    
        self.goalOrientation = {"x": 0, "y": 0, "z": 0}
        self.currentOrientation  = {"x": 0, "y": 0, "z": 0}

        self.panLimits = {"center": 68, "min":15, "max":115}
        self.tiltLimits = {"center": 44, "min":0, "max":88}
        self.servoStretching = 0.55

        self.dt = 0.02 #50 Hz
        self.newStateFlag = False
        self.newCommand = False
        self.exit = False
        self.position = [0.0,0.0]
        self.positionCommand = [0.0,0.0]
        self._controlLoopLock = RLock()

        #for pause/resume
        self.paused = False
        self.paused_command_sent = False
    def start(self):
        # dynamicel init
        # Open port
        if self.portHandler.openPort():
            print("Headcontroller: Succeeded to open the port")
        else:
            print("Headcontroller: Failed to open the port")
           
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Headcontroller: Succeeded to change the baudrate")
        else:
            print("Headcontroller:Failed to change the baudrate")
        # init position
        # self.ser.write(f'g,{68},{68}'.encode())
        self.dynamixel.write4ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(44/0.08789))
        self.dynamixel.write4ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(68/0.08789))
        time.sleep(0.5)
        controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()
        return True

    def _controlLoop(self):

        while not self.exit:
            ##update the state
            self._controlLoopLock.acquire()
            self.position = self._getHeadPosition()
            self.newStateFlag = True

            if self.paused:
                if not self.paused_command_sent:
                    self._setPosition(self.sensedPosition())      
                    self.paused_command_sent = True
            else:
                ##send command if there is a new one
                if self.newCommand:

                    self._setPosition(self.positionCommand)
                self.newCommand = False
            self._controlLoopLock.release()
            time.sleep(self.dt)
        print("Head Controller: control loop exited")

    def _getHeadPosition(self):
        """
        Return:
        ---------------
        A list of tilt and pan angles, in radians
        """
        dxl_present_position_tilt, dxl_comm_result_tilt, dxl_error_tilt = self.dynamixel.read4ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_PRESENT_POSITION)
        dxl_present_position_pan, dxl_comm_result_pan, dxl_error_pan = self.dynamixel.read4ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_PRESENT_POSITION)
        return [dxl_present_position_tilt*0.08789*DEGREE_2_RADIAN, dxl_present_position_pan*0.08789*DEGREE_2_RADIAN]

    def _setPosition(self, orientation):
        """
        Parameters:
        ---------------
        orientation:(tilt position, pan position) in radians
        
        Sets the motors to the specified location as soon as possible.
        """

        def mod180(x):
            while x >= 180:
                x = x - 360
            while x < -180:
                x = x + 360
            return x
        
        def limitTo(x,min,max):
            if x > max:
                return max
            elif x < min:
                return min
            return x
        
        self.goalOrientation["x"] = mod180(orientation[0]/DEGREE_2_RADIAN) #convert to angles
        self.goalOrientation["y"] = mod180(orientation[1]/DEGREE_2_RADIAN)

        # panAngle = limitTo((self.panLimits["center"] + self.goalOrientation["y"]*self.servoStretching), self.panLimits["min"], self.panLimits["max"])
        # tiltAngle = limitTo((self.tiltLimits["center"] + self.goalOrientation["x"]*self.servoStretching), self.tiltLimits["min"], self.tiltLimits["max"])

        panAngle = self.goalOrientation["y"]
        tiltAngle = self.goalOrientation["x"]

        # conversion degree/0.08789
        self.dynamixel.write4ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(tiltAngle/0.08789))
        self.dynamixel.write4ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(panAngle/0.08789))

    def setPosition(self,position):
        if not self.paused:
            self._controlLoopLock.acquire()
            self.positionCommand = copy(position)
            self.newCommand = True
            self._controlLoopLock.release()
        return

    def sensedPosition(self):
        return copy(self.position)

    def newState(self):
        """
        Return a new state is updated
        """

        return self.newStateFlag

    def markRead(self):
        self.newStateFlag = False

    def shutdown(self):
        self.exit = True

    def pause(self):
        self.paused = True
        self.paused_command_sent = False

    def resume(self):
        self.paused = False
        self.paused_command_sent = True

    def isPaused(self):
        return self.paused
        
if __name__ == "__main__":
    a = HeadController()
    a.start()
    time.sleep(1)
    print(a.sensedPosition())
    [pos1,pos2] = a.sensedPosition()
    a.setPosition([pos1,pos2+0.5])
    time.sleep(0.5)
    print(a.sensedPosition())
    a.shutdown()