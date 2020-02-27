from klampt import *
from klampt import vis
from klampt.vis.glrobotprogram import *
from klampt.math import *
from klampt.sim import *
from klampt.model import ik,coordinates,config
import time
import numpy as np
import trimesh
import os
import random
from copy import deepcopy

def get_top_grasp(mesh, link):
    # TODO: write a better grasp planner
    # now it just performances a top down grasp of random yaw rotation at the top center of the object
    min_grasp_height = 0.14
    L,W,H = mesh.bounding_box.extents
    # if W > 0.08:
    #     print("WARNING: item too large, likely cause gripper collision")
    #     selection = raw_input("CONTINUE? (y or n):")
    #     if selection is "n":
    #         exit()
    center = mesh.centroid.tolist()

    center[2] = max(min_grasp_height, H)

    center1 = deepcopy(center)
    center1[2] += 0.1
    # constrain 2 points to solve for a top down grasp
    objective = ik.objective(link,local = [[0, 0.14, 0],
                                           [0, 0.04, 0]],
                             world  = [center,center1] )

    Solver = ik.solver(objective)
    res = Solver.solve()
    return res



if __name__ == "__main__":

    # randomly choose a mesh to grasp from the models folder
    STATE = "idle"
    meshpaths = []
    for file in os.listdir("models"):
        if file.endswith(".ply"):
            meshpaths.append(os.path.join("models", file))

    meshpath = random.choice(meshpaths)
    mesh = trimesh.load(meshpath)

    mesh_center = mesh.centroid.tolist()
    Zmin = np.min(mesh.vertices[:,2])
    ee_driver_open_value = 0.0
    ee_driver_closed_value = 0.723
    world = WorldModel()

    # load world
    fn = "../shared_data/ur5e.xml"
    res = world.readFile(fn)


    robot = world.robot(0)
    link = robot.link(6)
    ee_pos = link.getWorldPosition([0,0,0])
    qhome = robot.getConfig()
    # make a template for the selected object
    with open('temp.obj', 'r') as template:
        data = template.readlines()
    data[1] = 'mesh '+ meshpath.replace(" ", "")+'\n'
    data[2] = 'mass '+ str(0.5)+'\n'

    # and write everything back
    with open('new.obj', 'w') as file:
        file.writelines(data)

    # load the rigid object in the world
    world.loadElement('new.obj')
    item = world.rigidObject(0)
    item.appearance().setColor(random.random(),random.random(),random.random(),1.0)
    bmass = item.getMass()
    bmass.setCom(mesh_center)
    item.setMass(bmass)
    item.setTransform([1,0,0,0,1,0,0,0,1],[ee_pos[0]-mesh_center[0],ee_pos[1]-mesh_center[1],-Zmin+0.02])
    program = GLSimulationPlugin(world)
    sim = program.sim

    #Set up simulation
    # body = sim.body(item)
    # sim_transform = body.getTransform()
    controller = sim.controller(0)
    program.simulate = True
    vis.setPlugin(program)
    vis.show()
    index = 0
    T_Start = time.time()
    while vis.shown():
        time.sleep(0.1)

        # give sometime for the object to settle
        if time.time() - T_Start > 8 and STATE == "idle":
            STATE = "pregrasp"
            print('State: pregrasp')

        if STATE == "pregrasp":

            mesh_copy = mesh.copy()
            R, T = item.getTransform()
            # convert to a 4 by 4 homogeneous transformation matrix
            transform = np.array([[R[0],R[3],R[6],T[0]],
                                  [R[1],R[4],R[7],T[1]],
                                  [R[2],R[5],R[8],T[2]],
                                  [0,0,0,1]])
            mesh_copy.apply_transform(transform)
            res = get_top_grasp(mesh_copy,link)
            if res:
                q = robot.getConfig()
                controller.setMilestone(q)
                time.sleep(2)
            STATE = "grasp"
            print('State: grasp')

        if STATE == "grasp":
            q = robot.getConfig()
            width = 0.1
            robot.driver(6).setValue(ee_driver_open_value*width + (1.0-width)*ee_driver_closed_value)
            qres = robot.getConfig()

            q_des = vectorops.interpolate(robot.getConfig(),qres,0.15)
            controller.setMilestone(q_des)

        if time.time() - T_Start > 14:
            q_res = robot.getConfig()
            STATE = "postpick"
            activeDofs = [0,1,2,3,4,5,6]
            for i in activeDofs:
                q_des[i] = qhome[i]
            controller.setMilestone(q_des)
            print('State: postpick')

        if time.time() - T_Start > 30:
            print('State: exiting')
            break
    vis.kill()
