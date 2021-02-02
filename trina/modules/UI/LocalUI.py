import sys
import os
import klampt
from klampt import vis
from klampt import io
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotModel,WorldModel,Simulator,RobotPoser
from klampt.model import ik,coordinates,config,trajectory,collide,sensing
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
import random
import weakref
try:
    import trina
except ImportError:  #run from command line?
    sys.path.append(os.path.expanduser("~/TRINA"))
    import trina
from trina import jarvis
from trina.modules.UI import UIAPI

glinit.init()
if glinit.available("PyQt"):
    if glinit.available("PyQt5"):
        from PyQt5.QtWidgets import *
        from PyQt5.QtGui import *
    else:
        from PyQt4.QtGui import *



# outer box
class MyQtMainWindow(QMainWindow):
    def __init__(self,klamptGLWindow,module):
        """When called, this be given a QtGLWidget object(found in klampt.vis.qtbackend).

        You will need to place this widget into your Qt window's layout.
        """
        QMainWindow.__init__(self)
        self.module = module
        self.currentApp = 'Manual'
        self.apps = ["Manual","DirectTeleOperation","PointClickNav","PointClickGrasp"]
        self.moveToPushButton = None
        self._initScreenLayout(klamptGLWindow)
        self._initMenuBar()
        self.setMouseTracking(True)

    def doRpc(self,func,args,kwargs,id):
        if func in ['getRayClick']:  #special deferred RPC calls
            return getattr(self,func)(id)
        else:
            kwarg_pairs = [str(k)+'='+str(v) for (k,v) in kwargs.items()]
            res = exec('self.{}({})'.format(fn,','.join(args+kwarg_pairs)))
            if id is not None:
                self.module.setRedisRpc(id,res)

    def closeEvent(self,event):
        if self.isVisible():
            reply = QMessageBox.question(self, "Confirm quit", "Do you really want to quit?",
                                    QMessageBox.Yes|QMessageBox.No)
            if reply == QMessageBox.Yes:
                vis.show(False)
                QMainWindow.close(self)
        else:
            QMainWindow.closeEvent(self,event)

    def addText(self,text,color,size,name): 
        vis.addText(name,text)
        self.module.screenElement.add(name)
        vis.setColor(name,color[0],color[1],color[2])
        vis.setAttribute(name,"size",size)
        return

    def addConfirmation(self,id,text,title):
        reply = QMessageBox.question(self, title, text,
                                    QMessageBox.Yes|QMessageBox.No)
        if reply == QMessageBox.Yes:
            return 'YES'
        else:
            return 'NO'

    def addPrompt(self,title,text):
        pass

    def addInputBox(self,title,text,fields):
        pass

    def sendTrajectory(self,trajectory,animate):
        world = self.module.world
        print("inside sendTrajectory")
        trajectory = io.loader.fromJson(trajectory,'Trajectory')
        print("pass from Json")
        robotPath = ("world",world.robot(0).getName())
        vis.add("robot trajectory",trajectory)
        self.screenElement.add("robot trajectory")

        if animate:
            vis.animate(robotPath,trajectory,endBehavior='halt')
        return

    def getRayClick(self,id):
        # ask the user to click the destination
        if self.module.global_state['collectRaySignal'] == [False,False]:
            reply = QMessageBox.question(self, "Click Navigation", "Please specify destination by right click on the map.",
                                        QMessageBox.Yes)
            if reply == QMessageBox.Yes:
                self.module.global_state['collectRaySignal'] = [True,False]
                self.module.global_state['feedback']['getRayClick'] = {}
                self.module.global_state['feedbackId']['getRayClick'] = id
            else:
                self.module.setRedisRpc(id,None)
        else:
            self.module.setRedisRpc(id,None)

    def addButton(self):
        name = self.rpc_args['name']
        text = self.rpc_args['text']
        self.buttons[name] = QPushButton(text)
        self.module.server['UI_STATE'][name] = True
        self.buttons[name].pressed.connect(lambda: self._handleButtonPress(name))
        self.buttons[name].released.connect(lambda: self._handleButtonRelease(name))
        self.appButtons.addWidget(self.buttons[name])
        return False

    def _handleButtonPress(self,id):
        self.module.server['UI_STATE'][name] = True

    def _handleButtonRelease(self,id):
        self.module.server['UI_STATE'][name] = True

    def _handleMoveToPress(self):
        self.module.moveToPoser()
        
    def _initMenuBar(self):
        bar = self.menuBar()
        mode = bar.addMenu("App Switch")
        mode.addAction("Manual")
        mode.addAction("PointClickNav")
        mode.addAction("PointClickGrasp")
        mode.addAction("DirectTeleOperation")
        mode.triggered[QAction].connect(self._processAppTrigger)

        action = bar.addMenu("Quick Action")
        action.addAction("Robot Stop")
        action.addAction("Quit")
        action.triggered[QAction].connect(self._processActionTrigger)

    def enableManualMode(self):
        robot = self.module.world.robot(0)
        vis.add('ghost',robot.getConfig(),color=(1,1,0,0.5))
        vis.edit('ghost')
        #TODO: connect to push button
        if self.moveToPushButton == None:
            self.moveToPushButton = QPushButton(' Move to')
            play_icon = QIcon.fromTheme('media-playback-start')
            self.moveToPushButton.setIcon(play_icon)
            self.moveToPushButton.pressed.connect(self._handleMoveToPress)
            self.debugLayout.addWidget(self.moveToPushButton)
        else:
            self.debugLayout.addWidget(self.moveToPushButton)

    def disableManualMode(self):
        vis.remove('ghost')
        self.moveToPushButton.setParent(None)

    def _processAppTrigger(self,q):
        print (q.text()+" is triggered")
        if self.currentApp == q.text():
            return
        print("Switching from",self.currentApp,"to",q.text())
        if q.text() == 'Manual':
            self.module.jarvis.deactivateModule("App_"+str(self.currentApp))
            self.currentApp = q.text()
            self.enableManualMode()
        else:
            if self.currentApp == 'Ma`nual':
                self.disableManualMode()
            self.currentApp = q.text()
            try:
                self.module.jarvis.switchModuleActivity(["App_"+str(self.currentApp)])
            except Exception as err:
                print("Error: {0}".format(err))
                import traceback
                traceback.print_exc()
        for element in self.module.screenElement:
            vis.remove(element)
        self.module.screenElement.clear()

    def _processActionTrigger(self,q):
        print (q.text()+" is triggered")
        if q.text() == "Emergency Stop":
            self.module.jarvis.stopMotion()
        elif q.text() == "Quit":
            self.module.terminate()
            self.close()

    def _onTabModeChanged(self,mode):
        if mode == 0:
            print("Switch to Debug mode")
            self.module.set_emulating(None)
        else:
            print("Switch to VR emulator mode")
            self.module.global_state['camera'] = 'fixed'
            self.module.set_emulating('vr')

    def _initScreenLayout(self,klamptGLWindow):
        self.setFixedSize(1580, 720)
        self.splitter = QSplitter()
        self.left = QTabWidget()
        self.debugFrame = QFrame()
        self.vrFrame = QFrame()
        self.left.addTab(self.debugFrame,"Debug")
        self.left.addTab(self.vrFrame,"VR emulator")
        self.left.setFixedSize(300,720)
        self.left.currentChanged.connect(self._onTabModeChanged)
        self.debugLayout = QVBoxLayout()
        self.debugFrame.setLayout(self.debugLayout)
        self.vrLayout = QVBoxLayout()
        self.vrFrame.setLayout(self.vrLayout)
        self.right = QFrame()
        self.right.setFixedSize(1280,720)
        self.rightLayout = QVBoxLayout()
        self.right.setLayout(self.rightLayout)
        
        self.glwidget = klamptGLWindow
        self.glwidget.do_reshape(1280,720)
        self.glwidget.setParent(self.right)
        self.rightLayout.addWidget(self.glwidget)

        self.buttons = {}
        self.appSelector = QListWidget()
        for i,appname in enumerate(self.apps):
            self.appSelector.addItem(appname)
        self.appSelector.setCurrentRow(0)
        self.appSelector.currentItemChanged.connect(self._processAppTrigger)
        
        self.debugLayout.addWidget(QLabel("Select mode/app:"))
        self.debugLayout.addWidget(self.appSelector)
    
        self.appButtons = QGroupBox("App buttons")
        self.appButtons2 = QGroupBox("App buttons")
        self.debugLayout.addWidget(self.appButtons)
        self.vrLayout.addWidget(self.appButtons2)

        #determine which camera sensors are available
        all_cams = ['fixed']
        i = 0
        while True:
            sensori = self.module.sim.controller(0).sensor(i)
            if sensori.type() == 'CameraSensor':
                all_cams.append(sensori.name())
            if sensori.type() == '':  #out of sensors
                break
            i += 1
        self.cameraSelector = QComboBox()
        self.all_cams = all_cams
        for cam in all_cams:
            self.cameraSelector.addItem(cam)
        def onSelectCamera(index):
            self.module.global_state["camera"] = self.all_cams[index]
        self.cameraSelector.activated.connect(onSelectCamera)
        self.debugLayout.addWidget(QLabel("Camera:"))
        self.debugLayout.addWidget(self.cameraSelector)

        #testing
        stop_icon = QIcon.fromTheme("process-stop")
        button = QPushButton(" Emergency Stop")
        button.setIcon(stop_icon)
        self.debugLayout.addWidget(button)

        self.enableManualMode()

        self.splitter.addWidget(self.left)
        self.splitter.addWidget(self.right)
        self.splitter.setHandleWidth(7)
        self.setCentralWidget(self.splitter)
    

# inner vis plugin
class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self,module):
        vis.GLPluginInterface.__init__(self)
        self.module = module
        self.world = module.world
        self.collider = collide.WorldCollider(self.world)

    def initialize(self):
        vis.GLPluginInterface.initialize(self)
        return True

    def mousefunc(self,button,state,x,y):
        if self.module.global_state['collectRaySignal'] == [False,False]:
            return False
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
        if c == 'w':
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"][1] = 1.0
        elif c == 's':
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"][1] = -1.0
        elif c == 'a':
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"][0] = -1.0
        elif c == 'd':
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"][0] = 1.0
        elif c == 'up':
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"][1] = 1.0
        elif c == 'down':
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"][1] = -1.0
        elif c == 'left':
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"][0] = -1.0
        elif c == 'right':
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"][0] = 1.0
        elif c == ' ':
            self.module.moveToPoser()
        else:
            return False
        self.module.server["UI_STATE"] = self.module.UIState 
        time.sleep(0.0001)
        return True

    def keyboardupfunc(self,c,x,y):
        print ("Released",c)
        leftJoystickMock = self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"]
        rightJoystickMock = self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"]
        if c == 'w' and leftJoystickMock[1] == 1.0:
            leftJoystickMock[1] = 0.0
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"] = leftJoystickMock 
        if c == 's' and leftJoystickMock[1] == -1.0:
            leftJoystickMock[1] = 0.0
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"] = leftJoystickMock 
        if c == 'a' and leftJoystickMock[0] == -1.0: 
            leftJoystickMock[0] = 0.0
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"] = leftJoystickMock 
        if c == 'd' and leftJoystickMock[0] == 1.0:
            leftJoystickMock[0] = 0.0
            self.module.UIState["controllerButtonState"]["leftController"]["thumbstickMovement"] = leftJoystickMock 
        if c == 'up' and rightJoystickMock[1] == 1.0:
            rightJoystickMock[1] = 0.0
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"] = rightJoystickMock 
        if c == 'down' and rightJoystickMock[1] == -1.0:
            rightJoystickMock[1] = 0.0
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"] = rightJoystickMock 
        if c == 'left' and rightJoystickMock[0] == -1.0: 
            rightJoystickMock[0] = 0.0
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"] = rightJoystickMock 
        if c == 'right' and rightJoystickMock[0] == 1.0:
            rightJoystickMock[0] = 0.0
            self.module.UIState["controllerButtonState"]["rightController"]["thumbstickMovement"] = rightJoystickMock 
        
        self.module.server["UI_STATE"] = self.module.UIState
        time.sleep(0.0001)
        return False

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)

        self._collect_ray(s,d)

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0],pt))

        return [g[1] for g in sorted(collided)]

    
    def _collect_ray(self, s, d, collided):
        try:
            id = self.module.global_state['feedbackId']['getRayClick']
            if self.module.global_state['collectRaySignal'][0]:
                vis.addText("pointclick","You have clicked the destination,\n Please indicate the direction.")
                vis.setColor("pointclick",1,0,0)
                vis.setAttribute("pointclick","size",30)
                vis.add("destination",sorted(collided)[0][2])
                self.module.screenElement.add("destination")

                vis.setColor("destination",1,0,0)
                vis.setAttribute("destination","size",10)

                self.module.global_state['feedback']['getRayClick'] = {'FIRST_RAY':{'source':[e for e in s], 'destination':[e for e in d]}}
                self.module.global_state['collectRaySignal'] = [False,True]

            elif self.module.global_state['collectRaySignal'][1]:
                vis.addText("pointclick","You have clicked the point for calibration")
                vis.setColor("pointclick",1,0,0)
                vis.setAttribute("pointclick","size",30)

                self.module.global_state['feedback']['getRayClick']['SECOND_RAY'] = {'source':[e for e in s], 'destination':[e for e in d]}
                self.module.setRedisRpc(id,self.global_state['feedback']['getRayClick'])
                self.module.global_state['collectRaySignal'] = [False,False]

                vis.remove("pointclick")

        except Exception as err:
            print("Error: {0}".format(err))



class LocalUI(jarvis.APIModule):
    """Runs a custom Qt frame around a visualization window"""
    def __init__(self,Jarvis=None):
        jarvis.APIModule.__init__(self,Jarvis)
        res = self.jarvis.require('Motion')  #requesting 'Motion' will provide the jarvis.robot accessor API.
        assert res != None
        res = self.jarvis.require('Sensor')  #requesting 'Sensor' will provide the jarvis.sensor accessor API.
        if res is None:
            self.sensors_enabled = False
        else:
            self.sensors_enabled = True
        if self.jarvis.robot.mode() == 'Kinematic':
            #self.world = trina.setup.simulation_world_load()
            self.world = trina.setup.robot_model_load()
        else:
            self.world = trina.setup.robot_model_load()
            #TODO: read perception data into vis
        assert self.world.numRobots() > 0
        self.verbose = 1
        self.sim = Simulator(self.world) #used for viewport simulation
        self.dt = 0.05
        self.UIState = {'controllerPositionState': {
                            'leftController': {
                                'controllerOrientation': [0.07739845663309097, -0.19212138652801514, 0.3228720426559448, 0.9235001802444458],
                                'controllerPosition': [-0.021801471710205078, -0.4208446145057678, 0.5902314186096191]
                            },
                            'rightController': {
                                'controllerOrientation': [0.052883781492710114, 0.20788685977458954, -0.30593231320381165, 0.927573025226593],
                                'controllerPosition': [0.15437912940979004, -0.4229428172111511, 0.5827353000640869]
                            }
                        }, 
                        'headSetPositionState': {
                            'deviceRotation': [-0.027466144412755966, 0.7671623826026917, 0.003965826239436865, 0.6408524513244629]
                        },
                        'controllerButtonState': {
                            'leftController': {
                                'nearTouch': [False, False],
                                'press': [False, False, False, False],
                                'thumbstickMovement': [0.0, 0.0],
                                'touch': [False, False, False, False, False, False, False, False],
                                'squeeze': [0.0, 0.0]
                            },
                            'rightController': {
                                'nearTouch': [False, False],
                                'press': [False, False, False, False],
                                'thumbstickMovement': [0.0, 0.0],
                                'touch': [False, False, False, False, False, False, False, False],
                                'squeeze': [0.0, 0.0]
                            }
                        },
                        'UIlogicState': {
                            'stop': False,
                            'autonomousMode': False,
                            'teleoperationMode': False
                        },
                        'title': 'UI Outputs'
                    }
        self.server = self.jarvis._state_server
        self.server["UI_STATE"] = self.UIState
        self.server["UI_END_COMMAND"] = []
        self.server['UI_FEEDBACK'] = {}
        self.global_state = {
            'collectRaySignal':[False,False],
            'feedbackId':{'getRayClick':''},
            'feedback':{},
            'camera':'fixed',
            }
        self.last_cam = 'fixed'
        self.screenElement = set([])
        self.emulating = None    #can be VR, tablet etc
        
        self._visInit()
        self.startSimpleThread(loopfunc = self._visLoop, dt = self.dt, name = "vis thread")

    @classmethod
    def name(self):
        return "UI"

    @classmethod
    def apiClass(self):
        return UIAPI

    def set_emulating(self,emulating):
        if self.emulating == emulating:
            return
        if self.emulating == None:
            if emulating == 'vr':
                try:
                    vis.remove('ghost')
                except Exception:
                    pass
                #z backward, x right, y up.  Rotate that to the frame of x forward, y left, z up
                #oculus_to_view = [[0,0,-1],
                #                  [-1,0,0],
                #                  [0,1,0]]
                #TODO: need to figure out how to convert rotations
                oculus_to_view = so3.matrix(so3.identity())
                Rheadset = so3.from_quaternion(self.UIState['headSetPositionState']['deviceRotation'])
                Rleft = so3.from_quaternion(self.UIState['controllerPositionState']['leftController']['controllerOrientation'])
                tleft = self.UIState['controllerPositionState']['leftController']['controllerPosition']
                Rright = so3.from_quaternion(self.UIState['controllerPositionState']['rightController']['controllerOrientation'])
                tright = self.UIState['controllerPositionState']['rightController']['controllerPosition']
                vis.add('vr_pose',(so3.mul(Rheadset,so3.from_matrix(oculus_to_view)),[0,0,2]))
                vis.add('vr_controller_left',(Rleft,tleft))
                vis.add('vr_controller_right',(Rright,tright))
                vis.edit('vr_pose')
                vis.edit('vr_controller_left')
                vis.edit('vr_controller_right')
                vis.hide('vr_pose',False)
                vis.hide('vr_controller_left',False)
                vis.hide('vr_controller_right',False)
        else:
            vis.remove('vr_pose')
            vis.remove('vr_controller_left')
            vis.remove('vr_controller_right')
            #TODO: re-enable app / manual mode

        self.emulating = emulating

    def _visInit(self):
        if not glinit.available("PyQt5"):
            print ("PyQt5 is not available on your system, try sudo apt-get install python-qt5")
            return
        
        # init qtmain window
        self.qt_mainwindow = None
        def makefunc(gl_backend):
            self.qt_mainwindow = MyQtMainWindow(gl_backend,weakref.proxy(self))
            return self.qt_mainwindow
        world = self.world

        sensed_position = self.jarvis.robot.sensedRobotq()
        world.robot(0).setConfig(sensed_position)

        vis.add("world",world)
        vis.hide(("world",world.robot(0).getName()))
        vis.add("robot_sensed",world.robot(0).getConfig())

        i = 0
        while True:
            sensori = self.sim.controller(0).sensor(i)
            if sensori.type() == 'CameraSensor':
                vis.add("Camera "+sensori.name(),sensori)
            if sensori.type() == '':  #out of sensors
                break
            i += 1

        vis.setWindowTitle("LocalUI")
        vis.customUI(makefunc)
        # init plugin for getting input
        plugin = MyGLPlugin(weakref.proxy(self))
        vis.pushPlugin(plugin)   #put the plugin on top of the standard visualization functionality.
        vis.show()

    def _visLoop(self):
        if not vis.shown():
            self.stopThread("vis thread")
        self.jarvis.log_health()

        world = self.world
        robot = world.robot(0)
        sensed_position = self.jarvis.robot.sensedRobotq()

        if self.emulating == None:
            vis.setItemConfig("robot_sensed",sensed_position)
            vis.lock()
            robot.setConfig(sensed_position)
            vis.unlock()
            cam = self.global_state['camera']
            if cam != 'fixed':
                if self.last_cam == 'fixed':
                    self.fixed_vp = vis.getViewport()
                #render from the camera's POV
                rgb = None
                if self.sensors_enabled:
                    res = self.jarvis.sensors.getRgbdImages(cam)[cam]
                    if res is not None:
                        if len(res)==2:
                            rgb = res[0]
                        else:
                            rgb = res
                        #print("Got a scene image with dimensions",rgb.shape,"dtype",rgb.dtype)
                        #print(" RGB range",rgb[:,:,0].min(),rgb[:,:,0].max(),rgb[:,:,1].min(),rgb[:,:,1].max(),rgb[:,:,2].min(),rgb[:,:,2].max())
            
                s = self.sim.controller(0).sensor(cam)
                vis.lock()
                svp = sensing.camera_to_viewport(s,robot)
                vp = vis.getViewport()
                svp.x = vp.x
                svp.y = vp.y
                svp.w = vp.w
                svp.h = vp.h
                svp.clippingplanes = vp.clippingplanes
                vis.setViewport(svp)
                if rgb is not None:
                    vis.scene().setBackgroundImage(rgb)
                vis.unlock()
            else:
                if self.last_cam != 'fixed':
                    vis.lock()
                    vis.scene().setBackgroundImage(None)
                    vis.setViewport(self.fixed_vp)
                    vis.unlock()
            self.last_cam = cam
        else:
            vis.setItemConfig("robot_sensed",sensed_position)
            vis.lock()
            robot.setConfig(sensed_position)
            vis.unlock()
            cam = self.global_state['camera']
            if cam != 'fixed':
                if self.last_cam == 'fixed':
                    self.fixed_vp = vis.getViewport()
                    vis.hide('vr_pose')
                    vis.hide('vr_controller_left')
                    vis.hide('vr_controller_right')
                #render from the camera's POV
                rgb = None
                if self.sensors_enabled:
                    res = self.jarvis.sensors.getRgbdImages(cam)[cam]
                    if res is not None:
                        if len(res)==2:
                            rgb = res[0]
                        else:
                            rgb = res
                        #print("Got a scene image with dimensions",rgb.shape,"dtype",rgb.dtype)
                        #print(" RGB range",rgb[:,:,0].min(),rgb[:,:,0].max(),rgb[:,:,1].min(),rgb[:,:,1].max(),rgb[:,:,2].min(),rgb[:,:,2].max())
            
                s = self.sim.controller(0).sensor(cam)
                vis.lock()
                svp = sensing.camera_to_viewport(s,robot)
                vp = vis.getViewport()
                svp.x = vp.x
                svp.y = vp.y
                svp.w = vp.w
                svp.h = vp.h
                svp.clippingplanes = vp.clippingplanes
                vis.setViewport(svp)
                if rgb is not None:
                    vis.scene().setBackgroundImage(rgb)
                vis.unlock()
            else:
                if self.last_cam != 'fixed':
                    vis.hide('vr_pose',False)
                    vis.hide('vr_controller_left',False)
                    vis.hide('vr_controller_right',False)
                    vis.lock()
                    vis.scene().setBackgroundImage(None)
                    vis.setViewport(self.fixed_vp)
                    vis.unlock()
                headset_pose = vis.getItemConfig('vr_pose')
                left_controller_pose = vis.getItemConfig('vr_controller_left')
                right_controller_pose = vis.getItemConfig('vr_controller_right')



        #process RPC calls
        for i in range(10):
            request = self.getRedisRpc()
            if request is None:
                break
            self.qt_mainwindow.doRpc(request['fn'],request['args'],request['kwargs'],request['id'])

    def moveToPoser(self):
        qdes = vis.getItemConfig('ghost')
        default_duration = 0
        qcur = self.jarvis.robot.sensedRobotq()    
        robot = self.world.robot(0)
        vmax = robot.getVelocityLimits()
        for i in range(robot.numLinks()):
            if vmax[i] > 0:
                default_duration = max(default_duration,abs(qcur[i]-qdes[i])/vmax[i])
        self.jarvis.robot.setKlamptPosition(qdes,default_duration)

    def terminate(self):
        # clean up
        vis.kill()
        del self.qt_mainwindow
        jarvis.APIModule.terminate(self)




if __name__ == "__main__":
    print ("""================================================================================
    LocalUI.py:

        USAGE: python3 LocalUI.py

    ================================================================================
    """)
    module = LocalUI()
    while module.status != 'terminated':
        time.sleep(1.0)
