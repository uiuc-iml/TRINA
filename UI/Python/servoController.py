import serial
import os
import time
import numpy as np                   # for multi-dimensional containers
import pandas as pd                  # for DataFrames
import plotly.graph_objects as go    # for data visualisation
import plotly.io as pio              # to set shahin plot layout
pio.templates['shahin'] = pio.to_templated(go.Figure().update_layout(margin=dict(t=0,r=0,b=40,l=40))).layout.template
pio.templates.default = 'shahin'
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_TORQUE_LIMIT       = 34
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_ACC_LIMIT          = 73

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID_tilt                 = 1                 # Dynamixel ID : 1, FOR PITCH 
DXL_ID_pan                  = 2                 # Dynamixel ID : 2, FOR YAW
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = 'COM3'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
# DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
# DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
# DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


class servoController:

    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.dynamixel = PacketHandler(PROTOCOL_VERSION)
        self.active = False    
        self.goalOrientation = {"x": 0, "y": 0, "z": 0}
        self.currentOrientation  = {"x": 0, "y": 0, "z": 0}
        self.startOrientation  = {"x": 0, "y": 0}

        self.panLimits = {"center": 180, "min":90, "max":270}
        self.tiltLimits = {"center": 180, "min":130, "max":230}


        self.servoStretching = 0.7


        # dynamicel init
        # Open port

        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        #SPEED
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_MOVING_SPEED, (int)(350))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_MOVING_SPEED, (int)(500))

        # init position
        # self.ser.write(f'g,{68},{68}'.encode())
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(self.tiltLimits["center"]/0.08789))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(self.panLimits["center"]/0.08789))

  


    def reportServoState(self):
        # print(self.ser.readline())
        dxl_present_position_tlit, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_PRESENT_POSITION)
        dxl_present_position_pan, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_PRESENT_POSITION)
        print('dxl_present_position_tlit', dxl_present_position_tlit*0.08789, 'dxl_present_position_pan',dxl_present_position_pan*0.08789)
    
    def initialize(self,orientation):
        self.startOrientation["x"] = orientation["x"]
        self.startOrientation["y"] = orientation["y"]
        return

    def setGoal(self, orientation):
        if self.startOrientation["x"] == 0:
            self.initialize(orientation)
            return
        
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
        
        self.goalOrientation["x"] = orientation["x"]
        self.goalOrientation["y"] = orientation["y"]

        panAngle = limitTo((self.panLimits["center"] - mod180(self.goalOrientation["y"] - self.startOrientation["y"])), self.panLimits["min"], self.panLimits["max"])
        tiltAngle = limitTo((self.tiltLimits["center"] + mod180(self.goalOrientation["x"] - self.startOrientation["x"])), self.tiltLimits["min"], self.tiltLimits["max"])
        # print(("moving ",panAngle,tiltAngle))
        # print(("startiing ",self.startOrientation["y"],self.startOrientation["x"]))

        # conversion degree/0.08789
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(tiltAngle/0.08789))
        self.dynamixel.write2ByteTxRx(self.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(panAngle/0.08789))
        # self.reportServoState()


if __name__ == "__main__":
    try:
        a = servoController()
        a.reportServoState()
        # sending the head sin waves in real timne
        sample_rate = 50
        sin_time = np.arange(0, 10, 1/sample_rate)
        sin_frequency = 1
        amplitude = 1
        theta = 0
        sinewave = amplitude * np.sin(2 * np.pi * sin_frequency * sin_time + theta)
        # fig = go.Figure(layout=dict(xaxis=dict(title='Time (sec)'),yaxis=dict(title='Amplitude')))
        # fig.add_scatter(x=time, y=sinewave)
        # fig.show()

        frequency = 60  # Hz
        period = 1.0/frequency


        for position in sinewave:
            time_before = time.time()
            tilt_position = position * (a.tiltLimits["max"] - a.tiltLimits["center"]) + a.tiltLimits["center"]
            print("tilt_position", tilt_position)
            # a.reportServoState()
            print("***********************************************************************")
            a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(tilt_position/0.08789))
        

            while (time.time() - time_before) < period:
                print(time.time() - time_before)
                time.sleep(0.001)  # precision here
        for position in sinewave:
            time_before = time.time()
            pan_position = position * (a.panLimits["max"] - a.panLimits["center"]) + a.panLimits["center"]
            # print("pan_position", pan_position)
            # a.reportServoState()
            # print("***********************************************************************")
            a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(pan_position/0.08789))
            while (time.time() - time_before) < period:
                print(time.time() - time_before)
                time.sleep(0.001)  # precision here



        # # moving the head up and down, left and right, end point position control
        # while True: 
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(130/0.08789))
        #     a.reportServoState()
        #     time.sleep(2)
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(230/0.08789))
        #     a.reportServoState()
        #     time.sleep(2)
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(180/0.08789))
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(180/0.08789))   
        #     a.reportServoState()
        #     time.sleep(2)
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(90/0.08789))
        #     a.reportServoState()
        #     time.sleep(2)
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(270/0.08789)) 
        #     a.reportServoState()
        #     time.sleep(2)
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_tilt, ADDR_MX_GOAL_POSITION, (int)(180/0.08789))
        #     a.dynamixel.write2ByteTxRx(a.portHandler, DXL_ID_pan, ADDR_MX_GOAL_POSITION, (int)(180/0.08789))   
        #     a.reportServoState()
        #     time.sleep(2)
    except KeyboardInterrupt:
        pass
          



   