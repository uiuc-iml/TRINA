from klampt import *
from klampt import WidgetSet,RobotPoser
from klampt.vis.glprogram import *
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import vis
from klampt.vis.qtbackend import QtBackend
from klampt.io import loader
from motion_client import MotionClient
import math
import time
import sys
from klampt.vis import glinit
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
    def __init__(self, world, parent,robot):
        GLWidgetProgram.__init__(self,"Manual poser")
        self.setParent(parent)
        self.world = world
        self.robot = robot
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
        #window.setMinimumSize(*VIEWER_MIN_SIZE)
        #window.setMaximumSize(*VIEWER_MAX_SIZE)
        window.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        window.idleTimer = QTimer()
        window.idleTimer.timeout.connect(lambda: self.idlefunc())
        window.idleTimer.start(0)
        window.setMouseTracking(True)
        # window.initialize()
        # window.initialized = True

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
        if currentTime >= self.saveStartTime and currentTime <= self.saveEndTime:
            self.leftEE = robot.sensedLeftEETransform()
            self.rightEE = robot.sensedRightEETransform()
            self.leftq = self.robot.sensedLeftLimbPosition()
            self.rightq = self.robot.sensedRightLimbPosition()
            self.saveMotionToCVS()
        robotModel.setConfig(q)
        robotModel.drawGL(False)
        glDisable(GL_BLEND)

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
        print("robot Current EE transform")
        self.leftEE = self.robot.sensedLeftEETransform()
        self.rightEE = self.robot.sensedRightEETransform()
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
        self.robot.setLeftLimbPositionLinear(leftUntuckedConfig,3)
        self.robot.setRightLimbPositionLinear(rightUntuckedConfig,3)
        print("setRobotToDefualt command")

    def resetPoser(self):
        self.robotPoser.set(self.robot.getKlamptSensedPosition())
        print("resetPoser command")




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
        self.setMinimumSize(QSize(760,760))
        #QMetaObject.invokeMethod(self.start_button, "clicked")

    def startMotionAPI(self):
        self.robot = MotionClient()
        res = self.robot.startup()
        if not res:
            return

        world = WorldModel()
        res = world.readFile("data/TRINA_world_reflex.xml")
        if not res:
            raise RuntimeError("Unable to load Klamp't model "+klampt_model_local)

        self.start_button.hide()

        self.viewer = MyGLViewer(world,self,self.robot)
        self.qtbackend = QtBackend() #klampt vis stuff for handing the poser

        self.qtbackend.app = QApplication.instance()
        viewer_window = self.qtbackend.createWindow("Control widget",self)
        self.viewer.window = viewer_window
        viewer_window.program = self.viewer
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
        PoseButton.resize(PoseButton.sizeHint())

        LogButton = QPushButton("Send and log pose", self)
        LogButton.clicked.connect(self.viewer.logPoseTrajectory)
        LogButton.resize(LogButton.sizeHint())

        EEbutton = QPushButton("Get current EE transform", self)
        EEbutton.clicked.connect(self.viewer.getSensedEETransform)
        EEbutton.resize(EEbutton.sizeHint())

        DefualtButton = QPushButton("Home position",self)
        DefualtButton.clicked.connect(self.viewer.setRobotToDefualt)
        DefualtButton.resize(DefualtButton.sizeHint())

        ResetPoserButton = QPushButton("Reset poser",self)
        ResetPoserButton.clicked.connect(self.viewer.resetPoser)
        ResetPoserButton.resize(ResetPoserButton.sizeHint())

        ToEETransformButton = QPushButton("To EE Transform",self)
        ToEETransformButton.clicked.connect(self.settoEETransform)
        ToEETransformButton.resize(ToEETransformButton.sizeHint())

        hbox = QHBoxLayout()
        hbox.addWidget(PoseButton)
        hbox.addWidget(LogButton)
        hbox.addWidget(EEbutton)
        hbox.addWidget(DefualtButton)
        hbox.addWidget(ResetPoserButton)
        hbox.addWidget(ToEETransformButton)
        self.layout().addLayout(hbox)

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


    def settoEETransform(self):
        leftR = [0.9996524663245526, 0.018497438182217656, 0.018265385647972483, -0.025855527200083765, 0.6346667994564419, 0.7723531256831561, 0.002694120347130487, -0.772558288058844, 0.6349231724662359]
        rightR = [0.9996524662307528, -0.018497444626183916, 0.018265384255842784, 0.025855530204947425, 0.6346667962295197, -0.7723531282342159, 0.0026941263138154643, 0.7725582905555143, 0.6349231694030447]
        input1, done1 = QInputDialog.getText( 
            self,'Left Arm Cartesian Operation', 'Enter x,y,z seperated by comma:')  
        leftT = input1.split(",")

        input2,  done2 = QInputDialog.getText( 
            self,'Right Arm Cartesian Operation', 'Enter x,y,z seperated by comma:')  
        rightT = input2.split(",")

        if done1:
            self.robot.setLeftEEInertialTransform([leftR,[float(i) for i in  leftT]],3)
        if done2:
            self.robot.setRightEEInertialTransform([rightR,[float(i) for i in  rightT]],3)
        print("settoEETransform")

def main():
    qapp = QApplication(sys.argv) ## this along with qapp will keep the GUI running after main is exited..
    cwidget = ControlWidget()
    cwidget.show()
    sys.exit(qapp.exec_())


if __name__ == '__main__':
    main()