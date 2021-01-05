import sys
import os
import klampt
from klampt import vis
from klampt import io
from klampt.robotsim import setRandomSeed
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import RobotPoser, Geometry3D
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
glinit.init()
sys.path.append(os.path.abspath('../../'))
import random
if not "--trigger" in sys.argv:
    # This import is really slow. Kinda jank but I want things to go fastfast.
    import trimesh

from Jarvis import Jarvis
if glinit.available("PyQt"):
    if glinit.available("PyQt5"):
        from PyQt5.QtWidgets import *
    else:
        from PyQt4.QtGui import *
import redis

model_path = '../../Resources/shared_data/objects/'
mesh_model_path = '../../Resources/grasping/models/'

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
        mode.addAction("PointClickGrasp")
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

class testingWorldBuilder():
    def __init__(self, floor_length=20, floor_width=20, world=[]):
        if (world == []):
            self.w = WorldModel()
        else:
            self.w = world
        self.floor = Geometry3D()
        self.floor.loadFile(model_path + "cube.off")
        self.floor.transform([floor_length, 0, 0, 0, floor_width, 0, 0, 0, 0.01],
                             [-floor_length / 2.0, -floor_width / 2.0, -0.01])
        #floor_terrain = self.w.makeTerrain("floor")
        #floor_terrain.geometry().set(self.floor)
        #floor_terrain.appearance().setColor(0.4, 0.3, 0.2, 1.0)

        ###colors
        self.light_blue = [3.0 / 255.0, 140.0 / 255.0, 252.0 / 255.0, 1.0]
        self.wall_green = [50.0 / 255.0, 168.0 / 255.0, 143.0 / 255.0, 1.0]
        ###sizes
        self.table_length = 1.2
        self.table_width = 0.8
        self.table_top_thickness = 0.03
        self.table_height = 0.8
        self.leg_width = 0.05
        self.cube_width = 0.05

    def getWorld(self):
        return self.w

    def addTableTopScenario(self, x=0, y=0):
        """
        add a table with objects on top, the center of table can be set
        Parameters:
        --------------
        x,y: floats, the position of the table center
        """

        self.addTable(x, y)
        # add some cubes
        self.addCube((so3.from_axis_angle(([0, 0, 1], 0.5)), [x, y - 0.7, self.table_height]), self.cube_width,
                     [1.0, 0, 0, 1], 1)
        # add one mesh
        random.seed(30)
        self.addRandomMesh([0 + x, -1.0 + y, self.table_height], 1)
        self.addRandomMesh([0 + x, -1.2 + y, self.table_height], 2)
        self.addRandomMesh([0.2 + x, -1.0 + y, self.table_height], 3)
        self.addRandomMesh([-0.2 + x, -1.2 + y, self.table_height], 4)

    def addIndoorNavScenario(self):
        """
        Add 4 rooms and a table
        """

        wall_thickness = 0.2
        room_size = [8.0, 6.0, 4.0]
        self.addRoom(room_size, wall_thickness, T=([1, 0, 0, 0, 1, 0, 0, 0, 1], [-6, -6, 0]), ID=1)
        self.addRoom(room_size, wall_thickness, T=([1, 0, 0, 0, 1, 0, 0, 0, 1], [6, -6, 0]), ID=2)
        self.addRoom(room_size, wall_thickness, T=(so3.from_axis_angle(([0, 0, 1], math.pi / 1.0)), [6, 6, 0]), ID=3)
        self.addRoom(room_size, wall_thickness, T=(so3.from_axis_angle(([0, 0, 1], math.pi / 1.0)), [-6, 6, 0]), ID=4)
        self.addTable(-6, -6)
        return

    def addRoom(self, room_size=[10.0, 8.0, 4.0], wall_thickness=0.2, T=([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 0, 0]), ID=1):
        """
        Add a room to the world
        Parameters:
        -------------------
        room_size = list of 3 floats, the width,depth, and height of the room
        wall_thickess = float
        T:rigid transform of the room
        ID: ID of the room
        """

        # left wall
        self._addTerrain(model_path + "cube.off",
                         se3.mul(T, ([wall_thickness, 0, 0, 0, room_size[1] - wall_thickness, 0, 0, 0, room_size[2]], \
                                     [-room_size[0] / 2.0, -room_size[1] / 2.0, 0])), self.wall_green,
                         "wall1_" + str(ID))
        # wall with window
        self._addTerrain(model_path + "cube.off", se3.mul(T, ([room_size[0], 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
                                                              [-room_size[0] / 2.0, -room_size[1] / 2.0, 0])),
                         self.wall_green, "wall2_" + str(ID))
        self._addTerrain(model_path + "cube.off", se3.mul(T, ([room_size[0], 0, 0, 0, wall_thickness, 0, 0, 0, 2], \
                                                              [-room_size[0] / 2.0, -room_size[1] / 2.0, 2])),
                         self.wall_green, "wall3_" + str(ID))
        self._addTerrain(model_path + "cube.off",
                         se3.mul(T, ([room_size[0] / 2.0 - 0.5, 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
                                     [-room_size[0] / 2.0, -room_size[1] / 2.0, 1])), self.wall_green,
                         "wall4_" + str(ID))
        self._addTerrain(model_path + "cube.off",
                         se3.mul(T, ([room_size[0] / 2.0 - 0.5, 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
                                     [0.5, -room_size[1] / 2.0, 1])), self.wall_green, "wall5_" + str(ID))

        # right wall
        self._addTerrain(model_path + "cube.off",
                         se3.mul(T, ([wall_thickness, 0, 0, 0, room_size[1] - wall_thickness, 0, 0, 0, room_size[2]], \
                                     [room_size[0] / 2.0 - wall_thickness, -room_size[1] / 2.0, 0])), self.wall_green,
                         "wall6_" + str(ID))

        # the wall with door
        self._addTerrain(model_path + "cube.off",
                         se3.mul(T, ([room_size[0] - 2.5, 0, 0, 0, wall_thickness, 0, 0, 0, room_size[2]], \
                                     [-room_size[0] / 2.0, room_size[1] / 2.0 - wall_thickness, 0])), self.wall_green,
                         "wall7_" + str(ID))
        self._addTerrain(model_path + "cube.off", se3.mul(T, ([1, 0, 0, 0, wall_thickness, 0, 0, 0, room_size[2]], \
                                                              [-room_size[0] / 2.0 + (room_size[0] - 1),
                                                               room_size[1] / 2.0 - wall_thickness, 0])),
                         self.wall_green, "wall8_" + str(ID))
        self._addTerrain(model_path + "cube.off", se3.mul(T, ([1.5, 0, 0, 0, wall_thickness, 0, 0, 0, 1], \
                                                              [-room_size[0] / 2.0 + (room_size[0] - 1 - 1.5),
                                                               room_size[1] / 2.0 - wall_thickness, 3])),
                         self.wall_green, "wall9_" + str(ID))

    ##Functions below add individual objects
    def addCube(self, T, side_length, color, ID, object_mass=0.1):
        """
        Add a cube to the world.
        Parameters:
        --------------
        T:world transform of the cube
        side_length: float, size of the cube
        color: RGBA values, (0-1)
        ID: int, cannot duplicate
        mass:object mass added at the object geometric center
        """

        self._addRigidObject(model_path + "cube.off",
                             ([side_length, 0, 0, 0, side_length, 0, 0, 0, side_length, ], [0, 0, 0]), T, \
                             color, object_mass, [side_length / 2.0, side_length / 2.0, side_length / 2.0],
                             "cube" + str(ID))

    def addMesh(self, path, T, color, mass, ID):
        """
        Add a mesh to the world.
        Parameters:
        --------------
        path: path to the mesh file
        T:world transform of the mesh
        color: RGBA values, (0-1)
        mass:object mass added at the object geometric center
        ID: int, cannot duplicate
        """
        mesh = trimesh.load(path)
        mesh_center = mesh.centroid.tolist()

        # load the rigid object in the world
        self._addRigidObject(path, ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0] * 3), T, \
                             color, mass, mesh_center, "item" + str(ID))

    def addRandomMesh(self, t, ID=1):
        """
        Add a household item to the world, randonmly selected from the library.
        Color is also determined randomly. Mass set to 1kg arbitrarily
        Parameters:
        --------------
        t:world position of the mesh
        ID: int, cannot duplicate
        """
        meshpaths = []
        for file in os.listdir(mesh_model_path):
            if file.endswith(".ply"):
                meshpaths.append(os.path.join(mesh_model_path, file))
        meshpath = random.choice(meshpaths)
        mesh = trimesh.load(meshpath)
        mesh_center = mesh.centroid.tolist()
        # Z_min = np.min(mesh.vertices[:,2])

        # t[2] = t[2]+mesh_center[2]-Z_min
        # load the rigid object in the world
        self._addRigidObject(meshpath, ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0] * 3), ([1, 0, 0, 0, 1, 0, 0, 0, 1], t), \
                             (random.random(), random.random(), random.random(), 1.0), 0.1, mesh_center,
                             "item" + str(ID))

    def addTable(self, x=0, y=0):
        """
        Add a table to the world
        Parameters:
        --------------
        x,y: floats, the x,y position of the center of the table
        """
        table_top = Geometry3D()
        table_leg_1 = Geometry3D()
        table_leg_2 = Geometry3D()
        table_leg_3 = Geometry3D()
        table_leg_4 = Geometry3D()

        table_top.loadFile(model_path + "cube.off")
        table_leg_1.loadFile(model_path + "cube.off")
        table_leg_2.loadFile(model_path + "cube.off")
        table_leg_3.loadFile(model_path + "cube.off")
        table_leg_4.loadFile(model_path + "cube.off")

        table_top.transform([self.table_length, 0, 0, 0, self.table_width, 0, 0, 0, \
                             self.table_top_thickness], [0, 0, self.table_height - self.table_top_thickness])
        table_leg_1.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height \
                               - self.table_top_thickness], [0, 0, 0])
        table_leg_2.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height - \
                               self.table_top_thickness], [self.table_length - self.leg_width, 0, 0])
        table_leg_3.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height -
                               self.table_top_thickness],
                              [self.table_length - self.leg_width, self.table_width - self.leg_width, 0])
        table_leg_4.transform([self.leg_width, 0, 0, 0, self.leg_width, 0, 0, 0, self.table_height -
                               self.table_top_thickness], [0, self.table_width - self.leg_width, 0])

        table_geom = Geometry3D()
        table_geom.setGroup()
        for i, elem in enumerate([table_top, table_leg_1, table_leg_2, table_leg_3, table_leg_4]):
            g = Geometry3D(elem)
            table_geom.setElement(i, g)
        table_geom.transform([0, -1, 0, 1, 0, 0, 0, 0, 1], [x - self.table_length / 2.0, y - self.table_width / 2.0, 0])
        table = self.w.makeTerrain("table")
        table.geometry().set(table_geom)
        table.appearance().setColor(self.light_blue[0], self.light_blue[1], self.light_blue[2], self.light_blue[3])

    def addRobot(self, path, T):
        """
        Add a robot to the world. You can directly use Klampt functions to add to the world as well
        Parameters:
        ------------
        path: path to the robot model
        T: transform of the base of the robot
        """

        world.loadElement(path)
        item = world.rigidObject(0)
        item.setTransform([1, 0, 0, 0, 1, 0, 0, 0, 1],
                          [ee_pos[0] - mesh_center[0], ee_pos[1] - mesh_center[1], -Zmin + 0.02])

    def _addRigidObject(self, path, T_g, T_p, color, object_mass, Com, name):
        item_1_geom = Geometry3D()
        item_1_geom.loadFile(path)
        item_1_geom.transform(T_g[0], T_g[1])
        item_1 = self.w.makeRigidObject(name)
        item_1.geometry().set(item_1_geom)
        item_1.appearance().setColor(color[0], color[1], color[2], color[3])
        bmass = item_1.getMass()
        bmass.setMass(object_mass)
        bmass.setCom(Com)
        item_1.setMass(bmass)
        item_1.setTransform(T_p[0], T_p[1])
        return item_1

    def _addTerrain(self, path, T, color, name):
        item_1_geom = Geometry3D()
        item_1_geom.loadFile(path)
        item_1_geom.transform(T[0], T[1])
        item_1 = self.w.makeTerrain(name)
        item_1.geometry().set(item_1_geom)
        item_1.appearance().setColor(color[0], color[1], color[2], color[3])
        return item_1

class UI_end_1:
    def __init__(self, placeholder, args):
        self.dt = 0.05
        self.UIState = {'controllerPositionState': {'leftController': {'controllerOrientation': [0.07739845663309097, -0.19212138652801514, 0.3228720426559448, 0.9235001802444458], 'controllerPosition': [-0.021801471710205078, -0.4208446145057678, 0.5902314186096191]}, 'rightController': {'controllerOrientation': [0.052883781492710114, 0.20788685977458954, -0.30593231320381165, 0.927573025226593], 'controllerPosition': [0.15437912940979004, -0.4229428172111511, 0.5827353000640869]}}, 'headSetPositionState': {'deviceRotation': [-0.027466144412755966, 0.7671623826026917, 0.003965826239436865, 0.6408524513244629]}, 'controllerButtonState': {'leftController': {'nearTouch': [False, False], 'press': [False, False, False, False], 'thumbstickMovement': [0.0, 0.0], 'touch': [False, False, False, False, False, False, False, False], 'squeeze': [0.0, 0.0]}, 'rightController': {'nearTouch': [False, False], 'press': [False, False, False, False], 'thumbstickMovement': [0.0, 0.0], 'touch': [False, False, False, False, False, False, False, False], 'squeeze': [0.0, 0.0]}}, 'UIlogicState': {'stop': False, 'autonomousMode': False, 'teleoperationMode': False}, 'title': 'UI Outputs'}
        self.interface = RedisInterface(host="localhost")
        self.interface.initialize()
        self.server = KeyValueStore(self.interface)
        self.server["UI_STATE"] = self.UIState
        self.server["UI_END_COMMAND"] = []
        self.server['UI_FEEDBACK'] = {}
        self.global_state = {'collectRaySignal':[False,False],'feedbackId':{'getRayClick':''}}
        self.jarvis = Jarvis(str("UI"),trina_queue = TrinaQueue(str("UI")))
        self.screenElement = set([])
        if "--teleop" in args:
            self.jarvis.changeActivityStatus(["DirectTeleOperation"])
        if "--testing" in args:
            self.jarvis.changeActivityStatus(["testing"])
        if not "--trigger" in args:
            file_dir = "../../Motion/data/TRINA_world_bubonic.xml"
            world = klampt.WorldModel()
            res = world.readFile(file_dir)
            builder = testingWorldBuilder(30,30,world = world)
            builder.addTableTopScenario(x = 1.5,y = 1.0)
            world = builder.getWorld()
            self.world = world
            if not res:
                raise RuntimeError("Unable to load model "+file_dir)
            self._serveVis()


    def _serveVis(self):
        """Runs a custom Qt frame around a visualization window"""
        if not glinit.available("PyQt5"):
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
                # print(self.jarvis.sensedBaseVelocity())
                # print(self.server["UI_STATE"]["controllerButtonState"]["rightController"]["thumbstickMovement"].read() )
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

class TrinaQueue(object):
	def __init__(self,key, host = 'localhost', port = 6379):
		self.r = redis.Redis(host = host, port = port)
		self.key = key
	def push(self,item):
		self.r.rpush(self.key,item) 



if __name__ == "__main__":
    print ("""================================================================================
    UI_end_1.py: powered by klampt vis
    
    Options:
        --trigger: Don't start the UI.
        --teleop: Send a command to start teleop mode. Use with --trigger for testing.
    """)
    UI_end_1("name", sys.argv)
