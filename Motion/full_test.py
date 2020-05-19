from motion import *
from klampt.math import vectorops,so2
from klampt import vis
from klampt import Geometry3D
import klampt
import math
import time
import matplotlib.pyplot as plt

from global_planner import *
from local_planner import *
from motion_primitives import *
from motion_profile import *
from utils import *
from geometry import *

import signal
import tf
import rospy
import sensor_msgs
from multiprocessing import Process, Pipe

def publish_tf(curr_pose):
    x, y, theta = curr_pose
    theta = theta % (math.pi*2)
    br = tf.TransformBroadcaster()
    br.sendTransform([x, y, 0], tf.transformations.quaternion_from_euler(0, 0, theta), rospy.Time.now(), "base_link", "odom")
    br.sendTransform([0.2, 0, 0.2], tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "base_scan", "base_link")

def publish_gmapping_stuff(conn):
    rospy.init_node("sensing_test_child")
    pub = rospy.Publisher("base_scan", sensor_msgs.msg.LaserScan)

    ros_msg = None
    curr_pose_child = None

    while True:
        if conn.poll():
            ros_msg, curr_pose_child = conn.recv()

        if ros_msg is not None and curr_pose_child is not None:
            pub.publish(ros_msg)
            publish_tf(curr_pose_child) 

def compute_global_path(conn, end, radius):
    preprocessed_gridmap = None

    while True:
        if conn.poll():
            preprocessed_gridmap, start = conn.recv()

        if preprocessed_gridmap is not None:
            dists, parents = navigation_function(preprocessed_gridmap, end, radius)
            global_path = get_path(parents, start, end)
            conn.send(global_path)

robot = Motion(mode = 'Kinematic', codename="anthrax")
world = robot.getWorld()

add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [2, 2, 0]), "test")
add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-2, 2, 0]), "test1")
add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [2, -2, 0]), "test2")
add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-2, -2, 0]), "test3")

add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, 5, 0]), "test")
add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [5, 0, 0]), "test1")
add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [0, -5, 0]), "test2")
add_terrain(world, "./data/cube.off", ([1, 0, 0, 0, 1, 0, 0, 0, 1], [-5, 0, 0]), "test3")

vis.add("world",world)

grid = get_occupancy_grid("static_map")
res = grid.info.resolution
radius = 0.5588/2/res * 2.0
gridmap = build_2d_map(grid)
preprocessed_gridmap = preprocess(gridmap, radius)

start = intify(transform_coordinates((0, 0), grid))
#end = intify(transform_coordinates((4, 4), grid))
#end = intify(transform_coordinates((4, 8), grid))
end = intify(transform_coordinates((8, 0), grid))
#end = intify(transform_coordinates((0, 8), grid))

curr_point = Circle(start[::-1], radius)
curr_theta = 0
end_theta = math.pi/2

leftTuckedConfig = [0.7934980392456055, -2.541288038293356, -2.7833543555, 4.664876623744629, -0.049166981373, 0.09736919403076172]
leftUntuckedConfig = [-0.2028,-2.1063,-1.610,3.7165,-0.9622,0.0974]
rightTuckedConfig = robot.mirror_arm_config(leftTuckedConfig)
rightUntuckedConfig = robot.mirror_arm_config(leftUntuckedConfig)

robot.startup()

# reset arms
robot.setLeftLimbPositionLinear(leftTuckedConfig,5)
robot.setRightLimbPositionLinear(rightTuckedConfig,5)

sim = klampt.Simulator(world)

lidar = sim.controller(0).sensor("lidar")

ros_parent_conn, ros_child_conn = Pipe()
gmapping_proc = Process(target=publish_gmapping_stuff, args=(ros_child_conn, ))
gmapping_proc.start()

global_map_parent_conn, global_map_child_conn = Pipe()
global_map_proc = Process(target=compute_global_path, args=(global_map_child_conn, end, radius))
global_map_proc.start()

rospy.init_node("sensing_test_parent")

def sigint_handler(sig, frame):
    vis.kill()
    robot.shutdown()
    gmapping_proc.terminate()
    gmapping_proc.join()
    sys.exit(1)

signal.signal(signal.SIGINT, sigint_handler)

vis.add("lidar", lidar)
vis.show()

end_v = 0.5
start_time = time.time()

pose_history = []

global_path = None

while True: 
    lidar.kinematicSimulate(world, 0.001)
    ros_msg = ros.to_SensorMsg(lidar, frame="/base_scan")
    curr_pose = robot.base_state.measuredPos
    ros_parent_conn.send([ros_msg, curr_pose]) 

    new_grid = get_occupancy_grid("dynamic_map", timeout=0.1)
    if new_grid is not None:
        grid = new_grid
        gridmap = build_2d_map(grid)
        preprocessed_gridmap = preprocess(gridmap, radius) 
        new_start = intify(transform_coordinates((curr_pose[0], curr_pose[1]), grid))
        global_map_parent_conn.send((preprocessed_gridmap, new_start))

    if global_map_parent_conn.poll():
        new_global_path = global_map_parent_conn.recv() 
        if new_global_path is not None:
            global_path = new_global_path

    if global_path is None:
        time.sleep(0.1)
        continue

    collision = curr_point.collides(gridmap)
    if collision:
        print("collided...this should not have happened")
        continue

    dist_to_goal = l2_dist(curr_point.center, end)
    if dist_to_goal < 0.2 / res:
        break

    # near goal, run a special controller?
    if dist_to_goal < 1.0 / res:
        primitives = [LocalPath([(curr_point.center[0], curr_point.center[1], curr_theta), (end[0], end[1], end_theta)])]
        end_v = 0
    else:
        primitives = get_primitives(curr_point.center, curr_theta, radius*2, radius*1)

    closest = evaluate_primitives(curr_point, primitives, global_path, gridmap)
    kglobal = klampt.model.trajectory.Trajectory(milestones = [transform_back([x, y], grid) for x, y in zip(global_path.get_xs(), global_path.get_ys())])


    vis.add("kglobaltraj", kglobal)
    if len(pose_history) > 0:
        khistory_traj = klampt.model.trajectory.Trajectory(milestones = [p for p in pose_history])
        vis.add("khistorytraj", khistory_traj)
        vis.setColor("khistorytraj", 0, 255, 0)

    if closest is None:
        print("No prim!!!")
        robot.setBaseVelocity([0.0, 0.4])
        new_pose = robot.base_state.measuredPos
        new_pose = transform_coordinates(new_pose, grid)

        curr_point = Circle((new_pose[0], new_pose[1]), radius)
        curr_theta = new_pose[2]
        continue

    xs, ys, thetas = closest.get_xytheta(200)

    plt.cla()
    plt.plot(global_path.get_xs(), global_path.get_ys())
    for p in primitives:
        px, py, _ = p.get_xytheta(200)
        plt.plot(px, py, color='r')
    plt.plot(xs, ys, color='g')
    plt.imshow(gridmap)
    curr_point.plot(plt)
    plt.pause(0.01)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.draw()

    acc = 0.5
    max_v = 0.5
    profile = generate_profile(closest, acc/res, max_v/res, 0.01, end_v = end_v/res, start_v=robot.base_state.measuredVel[0]/res)
    end_v = profile[-1].v
    milestones = [transform_back([x, y], grid) for x, y in zip(xs, ys)]
    ktraj = klampt.model.trajectory.Trajectory(milestones = milestones)

    N = len(profile)
    time_thresh = 0.2
    start_time = time.time()

    for i in range(N):
        iteration_start = time.time()
        state = profile[i]

        if iteration_start - start_time > time_thresh:
            end_v = min(max_v, end_v)
            break

        lidar.kinematicSimulate(world, 0.001)
        ros_msg = ros.to_SensorMsg(lidar, frame="/base_scan")
        pose = robot.base_state.measuredPos
        ros_parent_conn.send([ros_msg, pose])

        pose_history.append([pose[0], pose[1]])

        pose = transform_coordinates(pose, grid)

        curr_target = (state.x, state.y)

        trans_target = so2.apply(-pose[2], vectorops.sub(curr_target, (pose[0], pose[1])))

        linear_error = trans_target[0]
        cross_track_error = trans_target[1]

        k_linear = 1.0 * res
        k_angular = 1.0 * res

        vel = [state.v * res + k_linear*linear_error, state.w + k_angular*cross_track_error]

        vis.lock()
        vis.add("klocaltraj", ktraj)
        vis.setColor("klocaltraj", 0, 0, 255)
        robot.setBaseVelocity(vel)
        vis.unlock()

        new_pose = robot.base_state.measuredPos
        new_pose = transform_coordinates(new_pose, grid)
        curr_theta = new_pose[2]

        elapsed_time = time.time() - iteration_start
        remaining_time = max(0.01 - elapsed_time, 0.0)
        time.sleep(remaining_time)

    new_pose = robot.base_state.measuredPos
    new_pose = transform_coordinates(new_pose, grid)

    curr_point = Circle((new_pose[0], new_pose[1]), radius)
    curr_theta = new_pose[2]

robot.setBaseVelocity([0, 0])
vis.show()
