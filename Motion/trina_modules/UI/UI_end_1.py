import sys
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
if glinit._PyQtAvailable:
    if glinit._PyQt5Available:
        from PyQt5.QtWidgets import *
    else:
        from PyQt4.QtGui import *



# outer box
class MyQtMainWindow(QMainWindow):
    def __init__(self,klamptGLWindow,world,global_state,server):
        """When called, this be given a QtGLWidget object(found in klampt.vis.qtbackend).

        You will need to place this widget into your Qt window's layout.
        """

        self.commandQueue = []
        self.world = world
        self.global_state = global_state
        self.server = server


        QMainWindow.__init__(self)
        # Splitter to show 2 views in same widget easily.
        # self.resize(2000,2000)
        self.splitter = QSplitter()
        self.left = QFrame()
        self.leftLayout = QVBoxLayout()
        self.left.setLayout(self.leftLayout)
        self.right = QFrame()
        self.rightLayout = QVBoxLayout()
        self.right.setLayout(self.rightLayout)
        
        self.glwidget = klamptGLWindow
        self.glwidget.setParent(self.right)
        self.rightLayout.addWidget(self.glwidget)
    
        self.helloButton = QPushButton("Menu")
        self.leftLayout.addWidget(self.helloButton)

        self.splitter.addWidget(self.left)
        self.splitter.addWidget(self.right)
        self.splitter.setHandleWidth(7)
        self.setCentralWidget(self.splitter)
        self.dt = 0.05
        self.setMouseTracking(True)
        self.rpc_args = {}

    
    def event(self,event):
        try:
            self.commandQueue = self.server["UI_END_COMMAND"].read()
            if  self.commandQueue:
                command = self.commandQueue[0]
                try:
                    self.commandQueue.pop(0)
                    self.server["UI_END_COMMAND"] = self.commandQueue
                    self.rpc_args = command['args']
                    time.sleep(0.0001) 
                    exec('self.'+command['funcName']+'()')
                finally:
                    print("command recieved was " + command['funcName'])
                    print("-------------------------------------------------------------")
        except AttributeError as err:
            print("AttributeError: {0}".format(err))
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
        position =self.rpc_args['position']
        self.rpc_args = {}
        vis.addText(position,text)
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
        self.rpc_args = {}
        print("inside sendTrajectory")
        trajectory = io.loader.fromJson(trajectory,'Trajectory')
        print("pass from Json")
        robotPath = ("world",world.robot(0).getName())
        vis.add("robot trajectory",trajectory)
        vis.animate(robotPath,trajectory,endBehavior='halt')
        return

    def getRayClick(self):
        id = self.rpc_args['id']
        # ask the user to click the destination
        reply = QMessageBox.question(self, "CLick Navigation", "Please specify destination by right click on the map.",
                                    QMessageBox.Yes)
        if reply == QMessageBox.Yes:
            self.global_state['collectRaySignal'] = [True,False]
            self.global_state['feedbackId']['getRayClick'] = id
        

    def test(self):
        print("hello")

# inner vis plugin
class MyGLPlugin(vis.GLPluginInterface):
    def __init__(self,world,global_state,server):
        vis.GLPluginInterface.__init__(self)
        self.world = world
        self.collider = collide.WorldCollider(world)
        self.quit = False
        self.global_state = global_state
        self.server = server

        #adds an action to the window's menu
        def doquit():
            self.quit = True
        self.add_action(doquit,"Quit",'Ctrl+q',"Quit the program")

    def initialize(self):
        # vis.add("instructions1","Right-click to get the list of intersecting items")
        # vis.add("instructions2","Press q, Ctrl+q, or select Quit from the menu to quit")
        vis.GLPluginInterface.initialize(self)
        return True

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
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
        if c == 'q':
            self.quit = True
            return True
        return False


    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        id = self.global_state['feedbackId']['getRayClick']

        

        #run the collision tests
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = g[1].rayCast(s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0],pt))

        try:
            if self.global_state['collectRaySignal'][0]:
                print('you have clicked the destination')
                vis.addText("text4","you have clicked the destination,\n please click again for calibration")
                vis.setColor("text4",1,0,0)
                vis.setAttribute("text4","size",20)
                vis.add("destination",sorted(collided)[0][2])
                self.server['UI_FEEDBACK'][str(id)] = {'REPLIED':False, 'MSG':{'FIRST_RAY':{'source':[e for e in s], 'destination':[e for e in d]}}}
                self.global_state['collectRaySignal'] = [False,True]
            elif self.global_state['collectRaySignal'][1]:
                print('you have clicked the point for calibration')
                vis.addText("text4","you have clicked the point for calibration")
                vis.setColor("text4",1,0,0)
                vis.setAttribute("text4","size",24)
                self.server['UI_FEEDBACK'][str(id)]['MSG']['SECOND_RAY'] = {'source':[e for e in s], 'destination':[e for e in d]}
                time.sleep(0.001)
                self.server['UI_FEEDBACK'][str(id)]['REPLIED'] = True
                self.global_state['collectRaySignal'] = [False,False]
                vis.addText("text4","")

        except AttributeError as err:
            print("AttributeError: {0}".format(err))

        print("ray"," source: ", [e for e in s], "destination: ", [e for e in d])


        return [g[1] for g in sorted(collided)]



class UI_end_1:
    def __init__(self,placeholder):
        file_dir = "../../data/TRINA_world_seed.xml"
        world = klampt.WorldModel()
        res = world.readFile(file_dir)
        self.world = world
        if not res:
            raise RuntimeError("Unable to load model "+file_dir)
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["UI_STATE"] = 0
        self.global_state = {'collectRaySignal':[False,False],'feedbackId':{}}
        self._serveVis()


    def _serveVis(self):
        """Runs a custom Qt frame around a visualization window"""
        if not glinit._PyQtAvailable:
            print ("PyQt5 is not available on your system, try sudo apt-get install python-qt5")
            return
        world = self.world
        #Qt objects must be explicitly deleted for some reason in PyQt5...
        g_mainwindow = None
        #All Qt functions must be called in the vis thread.
        #To hook into that thread, you will need to pass a window creation function into vis.customUI.
        def makefunc(gl_backend):
            global g_mainwindow
            g_mainwindow = MyQtMainWindow(gl_backend,world, self.global_state, self.server)
            return g_mainwindow
        vis.add("world",world)
        vis.setWindowTitle("UI END 1")
        plugin = MyGLPlugin(world, self.global_state,self.server)
        vis.customUI(makefunc)
        vis.pushPlugin(plugin)   #put the plugin on top of the standard visualization functionality.
        vis.spin(float('inf'))
        vis.kill()

        #Safe cleanup of all Qt objects created in makefunc.
        #If you don't have this, PyQt5 complains about object destructors being called from the wrong thread
        del g_mainwindow

    



if __name__ == "__main__":
    print ("""================================================================================
    UI_end_1.py: powered by klampt vis
    """)
    UI_end_1("haha")