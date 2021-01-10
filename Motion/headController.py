egg_path='/usr/local/lib/python3.6/dist-packages/'
import sys
sys.path.append(egg_path)
import serial
import os
import math
import time
from threading import Thread, Lock, RLock
import threading
from copy import copy
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import dynamixel_sdk
print(dynamixel_sdk.__file__)
DEGREE_2_RADIAN = 2.0*math.pi/180.0

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_tilt                 = 1                 # Dynamixel ID : 1, FOR PITCH 
DXL_ID_pan                  = 2                 # Dynamixel ID : 2, FOR YAW
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'            # Check which port is being used on your controller
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
        self.dt = 0.05 #20 Hz
        self.newStateFlag = False
        self.newCommand = False
        self.exit = False
        self.position = [0.0,0.0]
        self.positionCommand = [0.0,0.0]
        self.panLimits = {"center": 180, "min":105, "max":255} #head limits
        self.tiltLimits = {"center": 180, "min":90, "max":230} #head limits
        self._controlLoopLock = RLock()

    def start(self):
        # dynamicel init
        # Open ports
        if self.portHandler.openPort():
            print("Headcontroller: Succeeded to open the port")
        else:
            print("Headcontroller: Failed to open the port")
           
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Headcontroller: Succeeded to change the baudrate")
        else:
            print("Headcontroller: Failed to change the baudrate")

        # Enable Torque 
        self.dynamixel.write1ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.dynamixel.write1ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        
        #SPEED
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_MOVING_SPEED, (int)(60))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_MOVING_SPEED, (int)(60))
        time.sleep(0.5)

        # init position
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(self.tiltLimits["center"]/0.08789))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(self.panLimits["center"]/0.08789))
        time.sleep(0.5)

        controlThread = threading.Thread(target = self._controlLoop)
        controlThread.start()
        return True

    def _controlLoop(self):

        while not self.exit:
            ##update the state
            self._controlLoopLock.acquire()
            # self.position = self._getHeadPosition()
            self.newStateFlag = True
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
        dxl_present_position_pan, dxl_comm_result_pan, dxl_error_pan = self.dynamixel.read2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_PRESENT_POSITION)
        dxl_present_position_tilt, dxl_comm_result_tilt, dxl_error_tilt = self.dynamixel.read2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_PRESENT_POSITION)

        return [dxl_present_position_pan*0.08789*DEGREE_2_RADIAN,dxl_present_position_tilt*0.08789*DEGREE_2_RADIAN]

    def _setPosition(self, orientation):
        """
        Parameters:
        ---------------
        orientation:(pan position, tilt position) in radians
        
        Sets the motors to the specified location as soon as possible.
        """

        
        panAngle = orientation[0]/DEGREE_2_RADIAN  #convert to angles
        tiltAngle = orientation[1]/DEGREE_2_RADIAN

        # conversion degree/0.08789
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(panAngle/0.08789))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(tiltAngle/0.08789))
        return True


    def setPosition(self,position):
        self._controlLoopLock.acquire()
        self.positionCommand = copy(position)
        self.newCommand = True
        self._controlLoopLock.release()
        return True

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
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(self.panLimits["center"]/0.08789))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(self.tiltLimits["max"]/0.08789))
        time.sleep(2)
        # Disable Dynamixel Torque
        self.dynamixel.write1ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.dynamixel.write1ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)

if __name__ == "__main__":
    a = HeadController()
    a.start()
    time.sleep(1)
    print(a.sensedPosition())
    # [pos1,pos2] = a.sensedPosition()
    # a.setPosition([pos1+1,pos2+1])
    time.sleep(0.5)
    
    a.shutdown()