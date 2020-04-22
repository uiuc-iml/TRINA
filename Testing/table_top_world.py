from klampt import WorldModel,Geometry3D
from klampt.model import sensing
import time
import matplotlib.pyplot as plt
import klampt
from klampt import vis
from klampt.math import so3,se3
import trimesh
import os
import random
import numpy as np
import math

model_path = "../Resources/shared_data/objects/"
mesh_model_path = "../Resources/grasping/models/"

class testingWorldBuilder():
    def __init__(self,floor_length = 20, floor_width = 20):
        self.w = WorldModel()
        self.floor =  Geometry3D()
        self.floor.loadFile(model_path+"cube.off")
        self.floor.transform([floor_length,0,0,0,floor_width,0,0,0,0.01],[-floor_length/2.0,-floor_width/2.0,-0.01])
        floor_terrain = self.w.makeTerrain("floor")
        floor_terrain.geometry().set(self.floor)
        floor_terrain.appearance().setColor(0.4,0.3,0.2,1.0)

        ###colors
        self.light_blue = [3.0/255.0, 140.0/255.0, 252.0/255.0,1.0]
        self.wall_green = [50.0/255.0, 168.0/255.0, 143.0/255.0,1.0]
        ###sizes
        self.table_length = 1.2
        self.table_width = 0.8
        self.table_top_thickness = 0.03
        self.table_height = 0.5
        self.leg_width = 0.05
        self.cube_width = 0.05

    def getWorld(self):
        return self.w

    def addTableTopScenario(self,x=0,y=0):
        """
        add a table with objects on top, the center of table can be set

        Parameters:
        --------------
        x,y: floats, the position of the table center

        """

        self.addTable(x,y)
        # add some cubes
        self.addCube((so3.from_axis_angle(([0,0,1],0.5)),[x/2,y,self.table_height]), self.cube_width, [1.0,0,0,1],1)
        # add one mesh
        self.addRandomMesh([0+x/2,0.2+y,self.table_height],1)
        self.addRandomMesh([0+x/2,-0.2+y,self.table_height],2)
        self.addRandomMesh([0.2+x/2,0+y,self.table_height],3)
        self.addRandomMesh([-0.2+x/2,-0.2+y,self.table_height],4)

    ##Functions below add individual objects
    def addCube(self,T,side_length,color,ID,object_mass = 0.1):
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

        self._addRigidObject(model_path + "cube.off",([side_length,0,0,0,side_length,0,0,0,side_length,],[0,0,0]),T,\
            color,object_mass,[side_length/2.0,side_length/2.0,side_length/2.0],"cube"+str(ID))

    def addMesh(self,path,T,color,mass,ID):
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
        self._addRigidObject(path,([1,0,0,0,1,0,0,0,1],[0]*3),T,\
            color,mass,mesh_center,"item"+str(ID))

    def addRandomMesh(self,t,ID = 1):
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
        #Z_min = np.min(mesh.vertices[:,2])

        #t[2] = t[2]+mesh_center[2]-Z_min
        # load the rigid object in the world
        self._addRigidObject(meshpath,([1,0,0,0,1,0,0,0,1],[0]*3),([1,0,0,0,1,0,0,0,1],t),\
            (random.random(),random.random(),random.random(),1.0),0.1,mesh_center,"item"+str(ID))

    def addTable(self,x=0,y=0):
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

        table_top.transform([self.table_length,0,0,0,self.table_width,0,0,0,\
            self.table_top_thickness],[0,0,self.table_height - self.table_top_thickness])
        table_leg_1.transform([self.leg_width,0,0,0,self.leg_width,0,0,0,self.table_height\
            - self.table_top_thickness],[0,0,0])
        table_leg_2.transform([self.leg_width,0,0,0,self.leg_width,0,0,0,self.table_height - \
            self.table_top_thickness],[self.table_length-self.leg_width,0,0])
        table_leg_3.transform([self.leg_width,0,0,0,self.leg_width,0,0,0,self.table_height - 
            self.table_top_thickness],[self.table_length-self.leg_width,self.table_width-self.leg_width,0])
        table_leg_4.transform([self.leg_width,0,0,0,self.leg_width,0,0,0,self.table_height - 
            self.table_top_thickness],[0,self.table_width-self.leg_width,0])

        table_geom = Geometry3D()
        table_geom.setGroup()
        for i,elem in enumerate([table_top,table_leg_1,table_leg_2,table_leg_3,table_leg_4]):
            g = Geometry3D(elem)
            table_geom.setElement(i,g)
        #table_geom.transform([1,0,0,0,1,0,0,0,1],[x-self.table_length/2.0,y-self.table_width/2.0,0])
        table_geom.transform(so3.from_rpy([0, 0, math.pi/2]),[x-self.table_length/2.0,y-self.table_width/2.0,0])
        table = self.w.makeTerrain("table")
        table.geometry().set(table_geom)
        table.appearance().setColor(self.light_blue[0],self.light_blue[1],self.light_blue[2],self.light_blue[3])

    def addRobot(self,path,T):
        """
        Add a robot to the world. You can directly use Klampt functions to add to the world as well

        Parameters:
        ------------
        path: path to the robot model
        T: transform of the base of the robot
        """

        self.w.loadRobot(path)
        rob = self.w.robot(0)
        #rob.link(3).setTransform(T[0], T[1])

    def _addRigidObject(self,path,T_g,T_p,color,object_mass,Com,name):
        item_1_geom = Geometry3D()
        item_1_geom.loadFile(path) 
        item_1_geom.transform(T_g[0],T_g[1])
        item_1 = self.w.makeRigidObject(name)
        item_1.geometry().set(item_1_geom)
        item_1.appearance().setColor(color[0],color[1],color[2],color[3])
        bmass = item_1.getMass()
        bmass.setMass(object_mass)
        bmass.setCom(Com)
        item_1.setMass(bmass)
        item_1.setTransform(T_p[0],T_p[1])
        return item_1

    def _addTerrain(self,path,T,color,name):
        item_1_geom = Geometry3D()
        item_1_geom.loadFile(path) 
        item_1_geom.transform(T[0],T[1])
        item_1 = self.w.makeTerrain(name)
        item_1.geometry().set(item_1_geom)
        item_1.appearance().setColor(color[0],color[1],color[2],color[3])
        return item_1


# TODO: arms still kinda block camera view
def reset_arms(robot):
    leftResetConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
    rightResetConfig = [0.2028,-1.0353,1.610,-0.5749,0.9622,-0.0974]
    config = [0]*7 + leftResetConfig + [0, 0] + rightResetConfig + [0]
    robot.setConfig(config)

# add table + robot
builder = testingWorldBuilder(30,30)
builder.addTableTopScenario(x=1.75,y=-0.2)
builder.addRobot("../Motion/data/robots/Anthrax.urdf", None)

w = builder.getWorld()
robot = w.robot(0)
reset_arms(robot)
sim = klampt.Simulator(w)

# setup simulated camera
cam = klampt.SimRobotSensor(sim.controller(0), "rgbd_camera", "CameraSensor")
# mount camera in place
cam.setSetting("link","4")
cam.setSetting("Tsensor","0.0 -0.707 0.707 -1.0 0.0 0.0  0 -0.707 -0.707   0.2 0 1.2")
# minimum range
cam.setSetting("zmin","0.1")
# x field of view
cam.setSetting("xfov","1.5")

# add to visualization
vis.add("world",w)
vis.add("cam", cam)

# Take a picture!
cam.kinematicSimulate(w, 0.01)
rgb, depth = sensing.camera_to_images(cam)
plt.imshow(rgb)
plt.show()

# run the simulation (indefinitely)
vis.run()
