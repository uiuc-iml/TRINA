from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.vis.glprogram import *
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import vis
from klampt.vis.qtbackend import QtBackend
from klampt.io import loader
from motion_client import MotionClient
import numpy as np
import math
import time
import sys
from klampt.vis import glinit
from scipy.spatial.transform import Rotation as R
if glinit._PyQt4Available:
    from PyQt4.QtGui import *
    from PyQt4.QtCore import *
    from PyQt4.QtOpenGL import *
    QWIDGETSIZE_MAX = ((1 << 24) - 1) # QWIDGETSIZE_MAX seems to be missing from pyqt4
else:
    from PyQt5.QtGui import *
    from PyQt5.QtWidgets import *
    from PyQt5.QtCore import *
    from PyQt5.QtOpenGL import *

import os
import datetime
import csv 

VIEWER_MIN_SIZE = (640,480) # width,height
VIEWER_MAX_SIZE = (QWIDGETSIZE_MAX,QWIDGETSIZE_MAX)
DRAW_FT_CALIBRATION = True
DRAW_FORCE_SCALE = 0.1
robot_ip = 'http://10.194.203.22:8080'

class GLWidgetProgram(GLPluginProgram):
    """A program that uses a widget plugin"""
    def __init__(self,name="GLWidgetProgram"):
        GLPluginProgram.__init__(self,name)
        self.widgetPlugin = GLWidgetPlugin()
    def initialize(self):
        GLPluginProgram.initialize(self)
        self.setPlugin(self.widgetPlugin)
    def addWidget(self,widget):
        self.widgetPlugin.addWidget(widget)

class MyGLViewer(GLWidgetProgram):
    def __init__(self, world, parent,robot,robot_info):
        GLWidgetProgram.__init__(self,"Manual poser")
        self.setParent(parent)
        self.world = world
        self.robot = robot
        self.robot_info = robot_info
        self.leftEE = []
        self.rightEE = []
        self.saveStartTime = datetime.datetime(2000, 5, 3) 
        self.saveEndTime = datetime.datetime(2000, 5, 3) 
        #read commanded configuration
        q = self.robot.getKlamptCommandedPosition()
        self.leftq = self.robot.sensedLeftLimbPosition()
        self.rightq = self.robot.sensedRightLimbPosition()
        world.robot(0).setConfig(q)
        self.robotPoser = RobotPoser(world.robot(0))
        self.addWidget(self.robotPoser)
        robot = world.robot(0)

    def setParent(self,parent):
        self.parent = parent

    def initWindow(self, parent=None, shared=None):
        """
        updates the viewer's window with initial values
        """
        window = self.window
        window.setFocusPolicy(Qt.StrongFocus)
        window.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        window.idleTimer = QTimer()
        window.idleTimer.timeout.connect(lambda: self.idlefunc())
        window.idleTimer.start(0)
        window.setMouseTracking(True)

    def run(self, parent=None):
        """ overrides the GLProgram run method to avoid using visualization.run """
        if parent == None:
            global _currentProgram
            _currentProgram = self.window
            app = QApplication([self.name])
        self.initWindow(parent)
        self.window.show()
        if parent == None:
            app.exec_()
            _currentProgram = None
            self.window.close()

    def display(self):
        # Put your display handler here
        # the current example draws the sensed robot in grey and the
        # commanded configurations in transparent green

        #this line will draw the world
        robot = self.robot
        robotModel = self.world.robot(0)
        q = robot.getKlamptSensedPosition()
        robotModel.setConfig(q)
        robotModel.drawGL()
        GLWidgetProgram.display(self)

        # draw commanded configuration
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,[0,1,0,0.5])
        q = robot.getKlamptCommandedPosition()
        currentTime = datetime.datetime.utcnow()
        self.leftEE = robot.sensedLeftEETransform()
        self.rightEE = robot.sensedRightEETransform()
        self.leftq = self.robot.sensedLeftLimbPosition()
        self.rightq = self.robot.sensedRightLimbPosition()
        if currentTime >= self.saveStartTime and currentTime <= self.saveEndTime:
            self.saveMotionToCVS()
        robotModel.setConfig(q)
        robotModel.drawGL(False)
        glDisable(GL_BLEND)
        self.robot_info.setText(
            'Left Joint Anles: (%f : %f : %f : %f : %f : %f)' % (self.leftq[0], self.leftq[1],self.leftq[2],self.leftq[3],self.leftq[4],self.leftq[5]) 
            + "\nLeft positions: (%f : %f : %f)" % (self.leftEE[1][0], self.leftEE[1][1],self.leftEE[1][2]) 
            +'\nRight Joint Anles: (%f : %f : %f : %f : %f : %f)' % (self.rightq[0], self.rightq[1],self.rightq[2],self.rightq[3],self.rightq[4],self.rightq[5]) 
            + "\nRight positions: (%f : %f : %f)" % (self.rightEE[1][0], self.rightEE[1][1],self.rightEE[1][2]) )

        ##not needed for now 
        # qdes = self.robotPoser.get()
        # if robotModel.distance(qdes,q) > 1e-3:
        #     #draw the movement
        #     ees = [robotModel.link(utilities.limb_joint_to_link(j,6)) for j in range(4)]
        #     eetraces = [[] for j in range(4)]
        #     L = robotModel.distance(qdes,q)
        #     N = int(math.ceil(L/0.1))
        #     for i in range(1,N):
        #         u = float(i)/(N+1)
        #         qinterp = robotModel.interpolate(q,qdes,u)
        #         qinterp[:6] = q[:6]
        #         robotModel.setConfig(qinterp)
        #         for j in range(4):
        #             eetraces[j].append(ees[j].getTransform()[1])
        #     glColor3f(0,1,0)
        #     glLineWidth(5.0)
        #     for j in range(4):
        #         glBegin(GL_LINE_STRIP)
        #         for t in eetraces[j]:
        #             glVertex3fv(t)
        #         glEnd()
        #     glLineWidth(1.0)

        #  no display forces; torque data unused
        pass
    def sendPose(self):
        q = self.robotPoser.get()
        left_limb_command = q[10:16]
        right_limb_command = q[35:41]
        #self.robot.setLeftLimbPosition(left_limb_command)
        self.robot.setLeftLimbPositionLinear(left_limb_command,3)
        self.robot.setRightLimbPositionLinear(right_limb_command,3)
        print("send position command")
        return


    def logPoseTrajectory(self):
        q = self.robotPoser.get()
        left_limb_command = q[10:16]
        right_limb_command = q[35:41]
        self.fileName = 'GUI_log/motion' + ''.join(str(e) for e in left_limb_command + right_limb_command) + '.csv'
        self.saveStartTime =  datetime.datetime.utcnow()
        self.saveEndTime = datetime.datetime.utcnow() + datetime.timedelta(0,3.1)
        fields = ['timestep', 'Left Shoulder', 'Left UpperArm', 'Left ForeArm', 'Left Wrist1','Left Wrist2','Left Wrist3','Right Shoulder', 'Right UpperArm', 'Right ForeArm', 'Right Wrist1','Right Wrist2','Right Wrist3','Left EE Transform', 'Right EE Transform' ] 
        with open(self.fileName, 'w') as csvfile: 
            # creating a csv writer object 
            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow(fields)
        self.robot.setLeftLimbPositionLinear(left_limb_command,3)
        self.robot.setRightLimbPositionLinear(right_limb_command,3)
        print("send position command")
        return

    def getSensedEETransform(self):
        msgBox = QMessageBox()
        msgBox.setWindowTitle("robot Current EE transform")
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.setText("left EE transform + right EE transform")
        msgBox.setInformativeText(("left R \n"+str(self.leftEE[0])+"\n\n"+"left T \n"+str(self.leftEE[1])+"\n\n"+"right R \n"+ str(self.rightEE[0])+"\n\n"+"right T \n"+str(self.rightEE[1])))
        msgBox.setStandardButtons(QMessageBox.Ok)
        msgBox.exec_()
        print("robot Current EE transform")
        print("----------------left EE transform---------------------")
        print(self.leftEE)
        print("----------------right EE transform---------------------")
        print(self.rightEE)


        # convert klampt configuration to limb command vector
        #self.sendMessage("Sending pose {}".format(q))
        #robot.all_limbs.positionCommand(robot.all_limbs.configFromKlampt(q))

    def sendMessage(self, msg):
        print msg
        # self.parentWidget().sendMessage(msg)


    def setRobotToDefualt(self):
        leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
        rightUntuckedConfig = self.robot.mirror_arm_config(leftUntuckedConfig)
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,10)
        self.robot.setRightLimbPositionLinear(rightUntuckedConfig,10)
        print("setRobotToDefualt command")

    def resetPoser(self):
        self.robotPoser.set(self.robot.getKlamptSensedPosition())
        print("resetPoser command")

    def leftTCPPosControl(self,direction,delta):
        newT = [float(i) for i in  self.leftEE[1]]
        if direction is 'x':
            newT[0] = newT[0] + delta
        elif direction is 'y':
            newT[1] = newT[1] + delta
        elif direction is 'z':
            newT[2] = newT[2] + delta
        self.robot.setLeftEEInertialTransform([self.leftEE[0],newT],0.1)
        print("change left arm " + direction + " by" + str(delta))


    def rightTCPPosControl(self,direction,delta):
        newT = [float(i) for i in  self.rightEE[1]]
        if direction is 'x':
            newT[0] = newT[0] + delta
        elif direction is 'y':
            newT[1] = newT[1] + delta
        elif direction is 'z':
            newT[2] = newT[2] + delta
        self.robot.setRightEEInertialTransform([self.rightEE[0],newT],0.1)
        print("change right arm " + direction + " by" + str(delta))
        

    def leftTCPRotControl(self,direction,delta):
        delta = math.radians(delta)
        newR = R.from_dcm(np.array(self.leftEE[0]).reshape((3,3)))

        if direction is 'x':
            newR = newR * R.from_dcm([[1,0,0],[0,math.cos(delta),math.sin(delta)],[0, -math.sin(delta), math.cos(delta)]])
        elif direction is 'y':
            newR = newR * R.from_dcm([[math.cos(delta),0,-math.sin(delta)],[0,1,0],[math.sin(delta),0,math.cos(delta)]])
        elif direction is 'z':
            newR = newR * R.from_dcm([[math.cos(delta),math.sin(delta),0],[-math.sin(delta),math.cos(delta),0],[0,0,1]])
        self.robot.setLeftEEInertialTransform([newR.as_dcm().flatten().tolist(),self.leftEE[1]],0.1)
        pass


    def rightTCPRotControl(self,direction,delta):
        delta = math.radians(delta)
        newR = R.from_dcm(np.array(self.rightEE[0]).reshape((3,3)))

        if direction is 'x':
            newR = newR * R.from_dcm([[1,0,0],[0,math.cos(delta),math.sin(delta)],[0, -math.sin(delta), math.cos(delta)]])
        elif direction is 'y':
            newR = newR * R.from_dcm([[math.cos(delta),0,-math.sin(delta)],[0,1,0],[math.sin(delta),0,math.cos(delta)]])
        elif direction is 'z':
            newR = newR * R.from_dcm([[math.cos(delta),math.sin(delta),0],[-math.sin(delta),math.cos(delta),0],[0,0,1]])
        self.robot.setRightEEInertialTransform([newR.as_dcm().flatten().tolist(),self.rightEE[1]],0.1)
        pass    






    def saveMotionToCVS(self):
        print(self.fileName)
        # print("----------------q-----------------")
        # print(self.q)
        # print("----------------left EE-----------------")
        # print(self.leftEE)
        # print("----------------rigth EE-----------------")
        # print(self.rightEE)
        with open(self.fileName, 'a') as csvfile: 
            # creating a csv writer object 
            csvwriter = csv.writer(csvfile) 
                
            # writing the fields 
            # csvwriter.writerow(fields) 
                
            # writing the data rows 
            csvwriter.writerow([datetime.datetime.utcnow() - self.saveStartTime] +self.leftq + self.rightq + [self.leftEE,self.rightEE] )

class ControlWidget(QWidget):
    def __init__(self, mode = 'Kinematic'):
        self.mode = mode
        super(ControlWidget,self).__init__()
        self.initUI()

    def initUI(self):
        self.start_button = QPushButton("Start", self)
        self.start_button.clicked.connect(self.startMotionAPI)
        self.start_button.resize(self.start_button.sizeHint())
        hbox = QHBoxLayout()
        vbox = QVBoxLayout()
        hbox.addWidget(self.start_button)
        vbox.addLayout(hbox)
        self.setLayout(vbox)
        self.setMinimumSize(QSize(860,960))

    def startMotionAPI(self):
        self.robot = MotionClient(robot_ip)
        self.robot.startServer(mode = 'Kinematic', components = ['left_limb'])
        res = self.robot.startup()
        if not res:
            return

        world = WorldModel()
        res = world.readFile("data/TRINA_world_reflex.xml")
        if not res:
            raise RuntimeError("Unable to load Klamp't model "+klampt_model_local)

        self.start_button.hide()
        self.robot_info = QLabel(self)
        self.viewer = MyGLViewer(world,self,self.robot,self.robot_info)
        self.qtbackend = QtBackend() #klampt vis stuff for handing the poser

        self.qtbackend.app = QApplication.instance()
        viewer_window = self.qtbackend.createWindow("Control widget",self)
        self.viewer.window = viewer_window
        viewer_window.program = self.viewer
        viewer_window.setMaximumWidth(860)
        self.layout().addWidget(viewer_window)

        self.createDisplays()
        self.posTimer = self.startTimer(1) # returns timer ID
        self.viewer.run(self)
        return

    #def timerEvent(self,event):
    #    if event.timerId() == self.posTimer:
    #        time.sleep(0.0001)
    #        print("timer activated")
    #    return
    def createDisplays(self):
        PoseButton = QPushButton("Send pose", self)
        PoseButton.clicked.connect(self.viewer.sendPose)

        LogButton = QPushButton("Send and log pose", self)
        LogButton.clicked.connect(self.viewer.logPoseTrajectory)

        EEbutton = QPushButton("Get current EE transform", self)
        EEbutton.clicked.connect(self.viewer.getSensedEETransform)

        DefualtButton = QPushButton("Home position",self)
        DefualtButton.clicked.connect(self.viewer.setRobotToDefualt)

        ResetPoserButton = QPushButton("Reset poser",self)
        ResetPoserButton.clicked.connect(self.viewer.resetPoser)

        ToEETransformButton = QPushButton("To EE Transform",self)
        ToEETransformButton.clicked.connect(self.toEETransform)


        ToJointAnglesButton = QPushButton("To Joint Angles",self)
        ToJointAnglesButton.clicked.connect(self.toJointAngles)

        LeftPosPlusXButton = QPushButton("+x",self)
        LeftPosPlusXButton.clicked.connect(self.leftPosPlusX)
        LeftPosPlusYButton = QPushButton("+y",self)
        LeftPosPlusYButton.clicked.connect(self.leftPosPlusY)
        LeftPosPlusZButton = QPushButton("+z",self)
        LeftPosPlusZButton.clicked.connect(self.leftPosPlusZ)
        LeftPosMinusXButton = QPushButton("-x",self)
        LeftPosMinusXButton.clicked.connect(self.leftPosMinusX)
        LeftPosMinusYButton = QPushButton("-y",self)
        LeftPosMinusYButton.clicked.connect(self.leftPosMinusY)
        LeftPosMinusZButton = QPushButton("-z",self)
        LeftPosMinusZButton.clicked.connect(self.leftPosMinusZ)

        LeftRotPlusXButton = QPushButton("+x",self)
        LeftRotPlusXButton.clicked.connect(self.leftRotPlusX)
        LeftRotPlusYButton = QPushButton("+y",self)
        LeftRotPlusYButton.clicked.connect(self.leftRotPlusY)
        LeftRotPlusZButton = QPushButton("+z",self)
        LeftRotPlusZButton.clicked.connect(self.leftRotPlusZ)
        LeftRotMinusXButton = QPushButton("-x",self)
        LeftRotMinusXButton.clicked.connect(self.leftRotMinusX)
        LeftRotMinusYButton = QPushButton("-y",self)
        LeftRotMinusYButton.clicked.connect(self.leftRotMinusY)
        LeftRotMinusZButton = QPushButton("-z",self)
        LeftRotMinusZButton.clicked.connect(self.leftRotMinusZ)



        RightPosPlusXButton = QPushButton("+x",self)
        RightPosPlusXButton.clicked.connect(self.rightPosPlusX)
        RightPosPlusYButton = QPushButton("+y",self)
        RightPosPlusYButton.clicked.connect(self.rightPosPlusY)
        RightPosPlusZButton = QPushButton("+z",self)
        RightPosPlusZButton.clicked.connect(self.rightPosPlusZ)
        RightPosMinusXButton = QPushButton("-x",self)
        RightPosMinusXButton.clicked.connect(self.rightPosMinusX)
        RightPosMinusYButton = QPushButton("-y",self)
        RightPosMinusYButton.clicked.connect(self.rightPosMinusY)
        RightPosMinusZButton = QPushButton("-z",self)
        RightPosMinusZButton.clicked.connect(self.rightPosMinusZ)


        RightRotPlusXButton = QPushButton("+x",self)
        RightRotPlusXButton.clicked.connect(self.rightRotPlusX)
        RightRotPlusYButton = QPushButton("+y",self)
        RightRotPlusYButton.clicked.connect(self.rightRotPlusY)
        RightRotPlusZButton = QPushButton("+z",self)
        RightRotPlusZButton.clicked.connect(self.rightRotPlusZ)
        RightRotMinusXButton = QPushButton("-x",self)
        RightRotMinusXButton.clicked.connect(self.rightRotMinusX)
        RightRotMinusYButton = QPushButton("-y",self)
        RightRotMinusYButton.clicked.connect(self.rightRotMinusY)
        RightRotMinusZButton = QPushButton("-z",self)
        RightRotMinusZButton.clicked.connect(self.rightRotMinusZ)



        self.robot_info.setText("hello!")
        infoBox = QVBoxLayout()
        infoBox.addWidget(self.robot_info)
        self.layout().addLayout(infoBox)

        LeftTCPBox = QGridLayout()
        LeftTCPBox.addWidget(QLabel("Left Arm"),1,1)
        LeftTCPBox.addWidget(QLabel("TCP Position Ctrl"),1,2)
        LeftTCPBox.addWidget(QLabel("TCP Rotation Ctrl"),1,5)
        LeftTCPBox.addWidget(LeftPosPlusZButton,2,1)
        LeftTCPBox.addWidget(LeftPosMinusXButton,2,2)
        LeftTCPBox.addWidget(LeftPosMinusZButton,2,3)
        LeftTCPBox.addWidget(LeftPosMinusYButton,3,1)
        LeftTCPBox.addWidget(LeftPosPlusYButton,3,3)
        LeftTCPBox.addWidget(LeftPosPlusXButton,4,2)
        LeftTCPBox.addWidget(LeftRotPlusZButton,2,4)
        LeftTCPBox.addWidget(LeftRotMinusXButton,2,5)
        LeftTCPBox.addWidget(LeftRotMinusZButton,2,6)
        LeftTCPBox.addWidget(LeftRotMinusYButton,3,4)
        LeftTCPBox.addWidget(LeftRotPlusYButton,3,6)
        LeftTCPBox.addWidget(LeftRotPlusXButton,4,5)
        self.layout().addLayout(LeftTCPBox)


        RightTCPBox = QGridLayout()
        RightTCPBox.addWidget(QLabel("Right Arm"),1,1)
        RightTCPBox.addWidget(QLabel("TCP Position Ctrl"),1,2)
        RightTCPBox.addWidget(QLabel("TCP Rotation Ctrl"),1,5)
        RightTCPBox.addWidget(RightPosPlusZButton,2,1)
        RightTCPBox.addWidget(RightPosMinusXButton,2,2)
        RightTCPBox.addWidget(RightPosMinusZButton,2,3)
        RightTCPBox.addWidget(RightPosMinusYButton,3,1)
        RightTCPBox.addWidget(RightPosPlusYButton,3,3)
        RightTCPBox.addWidget(RightPosPlusXButton,4,2)
        RightTCPBox.addWidget(RightRotPlusZButton,2,4)
        RightTCPBox.addWidget(RightRotMinusXButton,2,5)
        RightTCPBox.addWidget(RightRotMinusZButton,2,6)
        RightTCPBox.addWidget(RightRotMinusYButton,3,4)
        RightTCPBox.addWidget(RightRotPlusYButton,3,6)
        RightTCPBox.addWidget(RightRotPlusXButton,4,5)
        self.layout().addLayout(RightTCPBox)

        buttonBox = QGridLayout()
        buttonBox.addWidget(PoseButton,1,1)
        buttonBox.addWidget(LogButton,1,2)
        buttonBox.addWidget(EEbutton,1,3)
        buttonBox.addWidget(DefualtButton,1,4)
        buttonBox.addWidget(ResetPoserButton,2,1)
        buttonBox.addWidget(ToEETransformButton,2,2)
        buttonBox.addWidget(ToJointAnglesButton,2,3)
        self.layout().addLayout(buttonBox)

        return

    def closeEvent(self, event):
        # clean up before closing
        print "closing control"
        try:
            #self.killTimer(self.posTimer)
            print "killed control timer"
        except AttributeError:
            pass
        except Exception,err:
            print str(err)
        try:
            self.robot.shutdown()
            print "Shut down motion robot"
        except AttributeError:
            print "No motion robot found"
        except Exception,err:
            print str(err)


    def toEETransform(self):

        leftR = self.viewer.leftEE[0]
        rightR = self.viewer.rightEE[0]
        
        input1, done1 = QInputDialog.getText( 
            self,'Left Arm Cartesian Operation', 'Enter x,y,z seperated by comma:')  
        leftT = input1.split(",")

        input2,  done2 = QInputDialog.getText( 
            self,'Right Arm Cartesian Operation', 'Enter x,y,z seperated by comma:')  
        rightT = input2.split(",")

        if done1:
            msg_left = self.robot.setLeftEEInertialTransform([leftR,[float(i) for i in  leftT]],3)
            print(msg_left)
        if done2:
            msg_right = self.robot.setRightEEInertialTransform([rightR,[float(i) for i in  rightT]],3)
            print(msg_right)

        print("settoEETransform")

    def toJointAngles(self):

        input1, done1 = QInputDialog.getText( 
            self,'Left Arm joint angles', 'Enter q0,q1,q2,q3,q4,q5 in degrees seperated by comma:')  
        leftQ = input1.split(",")
        print(leftQ)

        input2,  done2 = QInputDialog.getText( 
            self,'Right Arm joint angles', 'Enter q0,q1,q2,q3,q4,q5 in degrees seperated by comma:')  
        rightQ = input2.split(",")
        print(rightQ)

        if done1:
            msg_left = self.robot.setLeftLimbPositionLinear([math.radians(float(i)) for i in leftQ],3)
            print(msg_left)
        if done2:
            msg_right = self.robot.setRightLimbPositionLinear([math.radians(float(i)) for i in rightQ],3)
            print(msg_right)
        
        print("settoJointAngles")
    
    def leftPosPlusZ(self):
        self.viewer.leftTCPPosControl('z',0.02)
    
    def leftPosMinusZ(self):
        self.viewer.leftTCPPosControl('z',-0.02)

    def leftPosPlusX(self):
        self.viewer.leftTCPPosControl('x',0.02)
    
    def leftPosMinusX(self):
        self.viewer.leftTCPPosControl('x',-0.02)
    
    def leftPosPlusY(self):
        self.viewer.leftTCPPosControl('y',0.02)

    def leftPosMinusY(self):
        self.viewer.leftTCPPosControl('y',-0.02)
    
    def rightPosPlusZ(self):
        self.viewer.rightTCPPosControl('z',0.02)
    
    def rightPosMinusZ(self):
        self.viewer.rightTCPPosControl('z',-0.02)
    
    def rightPosPlusX(self):
        self.viewer.rightTCPPosControl('x',0.02)
    
    def rightPosMinusX(self):
        self.viewer.rightTCPPosControl('x',-0.02)
    
    def rightPosPlusY(self):
        self.viewer.rightTCPPosControl('y',0.02)

    def rightPosMinusY(self):
        self.viewer.rightTCPPosControl('y',-0.02)



    def leftRotPlusZ(self):
        self.viewer.leftTCPRotControl('z',5)
    
    def leftRotMinusZ(self):
        self.viewer.leftTCPRotControl('z',-5)

    def leftRotPlusX(self):
        self.viewer.leftTCPRotControl('x',5)
    
    def leftRotMinusX(self):
        self.viewer.leftTCPRotControl('x',-5)
    
    def leftRotPlusY(self):
        self.viewer.leftTCPRotControl('y',5)

    def leftRotMinusY(self):
        self.viewer.leftTCPRotControl('y',-5)
    



    def rightRotPlusZ(self):
        self.viewer.rightTCPRotControl('z',5)
    
    def rightRotMinusZ(self):
        self.viewer.rightTCPRotControl('z',-5)
    
    def rightRotPlusX(self):
        self.viewer.rightTCPRotControl('x',5)
    
    def rightRotMinusX(self):
        self.viewer.rightTCPRotControl('x',-5)
    
    def rightRotPlusY(self):
        self.viewer.rightTCPRotControl('y',5)

    def rightRotMinusY(self):
        self.viewer.rightTCPRotControl('y',-5)

def main():
    qapp = QApplication(sys.argv) ## this along with qapp will keep the GUI running after main is exited..
    cwidget = ControlWidget()
    cwidget.show()
    sys.exit(qapp.exec_())


if __name__ == '__main__':
    main()