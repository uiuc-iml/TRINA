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
#api_root = os.environ.setdefault("RS_DUKE_API_PATH",os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..')))
#if __name__ == '__main__':
#    import sys
#    sys.path.append(api_root)

##import lcm
#import lcmwidget
#from RS import rsinter

#from Motion import motion
#from Config import system_config
#from Calibration import ft_calibration
#from Control import utilities

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
        #read commanded configuration
        q = self.robot.getKlamptSensedPosition()
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

        # convert klampt configuration to limb command vector
        #self.sendMessage("Sending pose {}".format(q))
        #robot.all_limbs.positionCommand(robot.all_limbs.configFromKlampt(q))

    def sendMessage(self, msg):
        print msg
        # self.parentWidget().sendMessage(msg)



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
        posebutton = QPushButton("Send pose", self)
        posebutton.clicked.connect(self.viewer.sendPose)
        posebutton.resize(posebutton.sizeHint())

        hbox = QHBoxLayout()
        hbox.addWidget(posebutton)
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
def main():
    qapp = QApplication(sys.argv) ## this along with qapp will keep the GUI running after main is exited..
    cwidget = ControlWidget()
    cwidget.show()
    sys.exit(qapp.exec_())


if __name__ == '__main__':
    main()