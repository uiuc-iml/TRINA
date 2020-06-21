import sys
import os
import klampt
from klampt import vis
from klampt import io
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser
from klampt.model import ik,coordinates,config,trajectory,collide
from klampt.math import vectorops,so3,se3
from klampt.vis import GLSimulationPlugin
import json
import time
import math
import threading
from threading import Thread
from reem.connection import RedisInterface
from reem.datatypes import KeyValueStore
from klampt.vis import glinit
sys.path.append(os.path.abspath('../../'))
from jarvis import Jarvis
if glinit._PyQtAvailable:
    if glinit._PyQt5Available:
        from PyQt5.QtWidgets import *
    else:
        from PyQt4.QtGui import *



# outer box
class MyQtMainWindow(QMainWindow):
    def __init__(self,klamptGLWindow,world,global_state,server,jarvis,UIState,screenElement):
        """When called, this be given a QtGLWidget object(found in klampt.vis.qtbackend).

        You will need to place this widget into your Qt window's layout.
        """
        QMainWindow.__init__(self)

        self.commandQueue = []
        self.world = world
        self.global_state = global_state
        self.jarvis = jarvis
        self.server = server
        self.dt = 0.05
        self.rpc_args = {}
        self.mode = ''
        self.UIState = UIState
        self.screenElement = screenElement


        self._initScrennLayout(klamptGLWindow)
        self._initMenuBar()
        self.setMouseTracking(True)

       

    def event(self,event):
        try:
            self.commandQueue = self.server["UI_END_COMMAND"].read()
            if  self.commandQueue:
                command = self.commandQueue[0]
                try:
                    self.commandQueue.pop(0)
                    self.server["UI_END_COMMAND"] = self.commandQueue
                    if command['from'] != self.mode:
                        print("ignoring command from " + command['from'] + "command recieved was" + command['funcName'])
                    else:
                        self.rpc_args = command['args']
                        time.sleep(0.0001) 
                        exec('self.'+command['funcName']+'()')
                except Exception as err:
                    print("Error: {0}".format(err))
                finally:
                    print("command recieved was " + command['funcName'])
                    print("-------------------------------------------------------------")
        except Exception as err:
                print("Error: {0}".format(err))
        QMainWindow.event(self,event)
        return 0





    def closeEvent(self,event):
        if self.isVisible():
            reply = QMessageBox.question(self, "Confirm quit", "Do you really want to quit?",
                                    QMessageBox.Yes|QMessageBox.No)
            if reply == QMessageBox.Yes:
                vis.show(False)
                QMainWindow.close(self)
        else:
            QMainWindow.closeEvent(self,event)

    def addText(self): 
        text =self.rpc_args['text']
        color =self.rpc_args['color']
        size =self.rpc_args['size']
        name =self.rpc_args['name']
        self.rpc_args = {}
        vis.addText(name,text)
        self.screenElement.add(name)
        vis.setColor(name,color[0],color[1],color[2])
        vis.setAttribute(name,"size",size)
        return

    def addConfirmation(self):
        id = self.rpc_args['id']
        text =self.rpc_args['text']
        title =self.rpc_args['title']
        self.rpc_args = {}
        reply = QMessageBox.question(self, title, text,
                                    QMessageBox.Yes|QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':True, 'MSG':'YES'}
        else:
            self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':True, 'MSG':'NO'}
            
        pass

    def addPrompt(self,title,text):
        pass

    def addInputBox(self,title,text,fields):
        pass

    def sendTrajectory(self):
        world = self.world
        trajectory = self.rpc_args['trajectory']
        animate = self.rpc_args['animate']
        self.rpc_args = {}
        print("inside sendTrajectory")
        trajectory = io.loader.fromJson(trajectory,'Trajectory')
        print("pass from Json")
        robotPath = ("world",world.robot(0).getName())
        vis.add("robot trajectory",trajectory)
        self.screenElement.add("robot trajectory")

        if animate:
            vis.animate(robotPath,trajectory,endBehavior='halt')
        return

    def getRayClick(self):
        id = self.rpc_args['id']
        # ask the user to click the destination
        reply = QMessageBox.question(self, "Click Navigation", "Please specify destination by right click on the map.",
                                    QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            self.global_state['collectRaySignal'] = [True,False]
            self.global_state['feedbackId']['getRayClick'] = id

    def addButton(self):
        name = self.rpc_args['name']
        text = self.rpc_args['text']
        id = '$' + name
        self.buttons[name] = QPushButton(text)
        self.buttons[name].clicked.connect(lambda: self._handleButtonClick(id))
        self.leftLayout.addWidget(self.buttons[name])
        self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':True, 'MSG':False}

    def _handleButtonClick(self,id):
        self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':True, 'MSG':True}





        
    def test(self):
        print("inde the test function")

    
    def _initMenuBar(self):
        bar = self.menuBar()
        mode = bar.addMenu("Mode Switch")
        mode.addAction("PointClickNav")
        mode.addAction("DirectTeleOperation")
        mode.triggered[QAction].connect(self._processModeTrigger)

        action = bar.addMenu("Quick Action")
        action.addAction("Emergency Stop")
        action.addAction("Pause")
        action.addAction("Quit")
        action.triggered[QAction].connect(self._processActionTrigger)

    def _processModeTrigger(self,q):
        print (q.text()+" is triggered")
        if self.mode != q.text():
            self.modeText.setText("Mode: " +  q.text())
            self.mode = q.text()
            try:
                self.jarvis.changeActivityStatus([str(self.mode)])
                for element in self.screenElement:
                    vis.hide(element)
                self.screenElement.clear()
            except Exception as err:
                print("Error: {0}".format(err))

    def _processActionTrigger(self,q):
        print (q.text()+" is triggered")
        if q.text() == "Quit":
            self.close()



    def _initScrennLayout(self,klamptGLWindow):
        self.setFixedSize(1000, 600)
        self.splitter = QSplitter()
        self.left = QFrame()
        self.left.setFixedSize(400,600)
        self.leftLayout = QVBoxLayout()
        self.left.setLayout(self.leftLayout)
        self.right = QFrame()
        self.right.setFixedSize(600,600)
        self.rightLayout = QVBoxLayout()
        self.right.setLayout(self.rightLayout)
        
        self.glwidget = klamptGLWindow
        self.glwidget.do_reshape(600,600)
        self.glwidget.setParent(self.right)
        self.rightLayout.addWidget(self.glwidget)
    
        self.buttons = {}
        self.welcomeText = QLabel("Welcome to TRINA UI! \n\n\nPlease Refer To Menu Bar Actions For Selecting Operation Mode.")
        self.modeText = QLabel("")
        self.modeText.setWordWrap(True)
        self.welcomeText.setWordWrap(True)
        
        self.leftLayout.addWidget(self.welcomeText)
        self.leftLayout.addWidget(self.modeText)

        self.splitter.addWidget(self.left)
        self.splitter.addWidget(self.right)
        self.splitter.setHandleWidth(7)
        self.setCentralWidget(self.splitter)
    

# inner vis plugin
class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self,world,global_state,server,jarvis,UIState,screenElement):
        vis.GLPluginInterface.__init__(self)
        self.world = world
        self.collider = collide.WorldCollider(world)
        self.quit = False
        self.global_state = global_state
        self.server = server
        self.jarvis = jarvis
        self.UIState = UIState
        self.screenElement = screenElement


    def initialize(self):
        vis.GLPluginInterface.initialize(self)
        return True

    def mousefunc(self,button,state,x,y):
        print ("mouse",button,state,x,y)
        if button==2:
            if state==0:
                print ("Click list...",[o.getName() for o in self.click_world(x,y)])
            return True
        return False

    def motionfunc(self,x,y,dx,dy):
        return False

    def keyboardfunc(self,c,x,y):
        print ("Pressed",c)
        rightJoystickMock = self.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"]
        if c == 'w':
            rightJoystickMock[1] = 1.0
        if c == 's':
            rightJoystickMock[1] = -1.0
        if c == 'a':
            rightJoystickMock[0] = -1.0
        if c == 'd':
            rightJoystickMock[0] = 1.0
        self.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"] = rightJoystickMock 
        self.server["UI_STATE"] = self.UIState 
        time.sleep(0.0001)
        if c == 'q':
            self.quit = True
            return True
        return False

    def keyboardupfunc(self,c,x,y):
        print ("Released",c)
        rightJoystickMock = self.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"]
        if c == 'w' and rightJoystickMock[1] == 1.0:
            rightJoystickMock[1] = 0.0
        if c == 's' and rightJoystickMock[1] == -1.0:
            rightJoystickMock[1] = 0.0
        if c == 'a' and rightJoystickMock[0] == -1.0: 
            rightJoystickMock[0] = 0.0
        if c == 'd' and rightJoystickMock[0] == 1.0:
            rightJoystickMock[0] = 0.0
        self.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"] = rightJoystickMock 
        self.server["UI_STATE"] = self.UIState
        time.sleep(0.0001)
        return False



    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0],pt))

        self._collect_ray(s,d,collided)
       
        return [g[1] for g in sorted(collided)]

    
    def _collect_ray(self, s, d, collided):
        try:
            id = self.global_state['feedbackId']['getRayClick']
            if self.global_state['collectRaySignal'][0]:
                vis.addText("pointclick","You have clicked the destination,\n Please click again for calibration.")
                self.screenElement.addInputBox("pointclick")
                vis.setColor("pointclick",1,0,0)
                vis.setAttribute("pointclick","size",30)
                vis.add("destination",sorted(collided)[0][2])
                self.screenElement.add("destination")


                vis.setColor("destination",1,0,0)
                vis.setAttribute("destination","size",10)


                self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':False, 'MSG':{'FIRST_RAY':{'source':[e for e in s], 'destination':[e for e in d]}}}
                self.global_state['collectRaySignal'] = [False,True]

            elif self.global_state['collectRaySignal'][1]:
                vis.addText("pointclick","You have clicked the point for calibration")
                vis.setColor("pointclick",1,0,0)
                vis.setAttribute("pointclick","size",30)

                self.server['UI_FEEDBACK'][str(id)]['MSG']['SECOND_RAY'] = {'source':[e for e in s], 'destination':[e for e in d]}
                time.sleep(0.001)
                self.server['UI_FEEDBACK'][str(id)]['REPLIED'] = True
                self.global_state['collectRaySignal'] = [False,False]

                vis.remove("pointclick")

        except Exception as err:
            print("Error: {0}".format(err))


class UI_end_1:
    def __init__(self,placeholder):
        # file_dir = "../../data/TRINA_world.xml"
        file_dir = "../../data/TRINA_world_anthrax_PointClick.xml"
        world = klampt.WorldModel()
        res = world.readFile(file_dir)
        self.world = world
        self.dt = 0.05
        self.UIState = {'controllerPositionState': {'leftController': {'controllerOrientation': [0.07739845663309097, -0.19212138652801514, 0.3228720426559448, 0.9235001802444458], 'controllerPosition': [-0.021801471710205078, -0.4208446145057678, 0.5902314186096191]}, 'rightController': {'controllerOrientation': [0.052883781492710114, 0.20788685977458954, -0.30593231320381165, 0.927573025226593], 'controllerPosition': [0.15437912940979004, -0.4229428172111511, 0.5827353000640869]}}, 'headSetPositionState': {'deviceRotation': [-0.027466144412755966, 0.7671623826026917, 0.003965826239436865, 0.6408524513244629]}, 'controllerButtonState': {'leftController': {'nearTouch': [False, False], 'press': [False, False, False, False], 'thumbstickMovement': [0.0, 0.0], 'touch': [False, False, False, False, False, False, False, False], 'squeeze': [0.0, 0.0]}, 'rightController': {'nearTouch': [False, False], 'press': [False, False, False, False], 'thumbstickMovement': [0.0, 0.0], 'touch': [False, False, False, False, False, False, False, False], 'squeeze': [0.0, 0.0]}}, 'UIlogicState': {'stop': False, 'autonomousMode': False, 'teleoperationMode': False}, 'title': 'UI Outputs'}
        if not res:
            raise RuntimeError("Unable to load model "+file_dir)
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["UI_STATE"] = self.UIState
        self.server["UI_END_COMMAND"] = []
        self.server['UI_FEEDBACK'] = {}
        self.global_state = {'collectRaySignal':[False,False],'feedbackId':{'getRayClick':''}}
        self.jarvis = Jarvis(str("UI"))
        self.screenElement = set([])
        self._serveVis()


    def _serveVis(self):
        """Runs a custom Qt frame around a visualization window"""
        if not glinit._PyQtAvailable:
            print ("PyQt5 is not available on your system, try sudo apt-get install python-qt5")
            return
        world = self.world

        # init qtmain window
        g_mainwindow = None
        def makefunc(gl_backend):
            global g_mainwindow
            g_mainwindow = MyQtMainWindow(gl_backend,world, self.global_state, self.server, self.jarvis, self.UIState, self.screenElement)
            return g_mainwindow
        vis.add("world",world)
        vis.setWindowTitle("UI END 1")
        vis.customUI(makefunc)
        # init plugin for getting input
        plugin = MyGLPlugin(world, self.global_state,self.server,self.jarvis, self.UIState,self.screenElement)
        vis.pushPlugin(plugin)   #put the plugin on top of the standard visualization functionality.
       


        vis.show()
        while vis.shown():
            vis.lock()
            try:
                sensed_position = self.server['ROBOT_STATE']['Position']['Robotq'].read()
                # print(sensed_position[:2])
                vis_robot = world.robot(0)
                vis_robot.setConfig(sensed_position)
            except Exception as err:
                print("Error: {0}".format(err))
            vis.unlock()
            time.sleep(self.dt)
        vis.kill()

        # start vis

        # vis.spin(float('inf'))
        # vis.kill()

        # clean up
        del g_mainwindow

    



if __name__ == "__main__":
    print ("""================================================================================
    UI_end_1.py: powered by klampt vis
    """)
    UI_end_1("name")