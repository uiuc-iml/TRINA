import rospy
import tf
from klampt import *
import klampt.io.ros
from klampt.model import sensing
from klampt.math import so3

rospy.init_node('testing')
w = WorldModel()
res = w.readFile("../Models/simulation_worlds/TRINA_world_bubonic_with_obstacle.xml")
assert res,"Couldn't read file?"
s = Simulator(w)
lidar = s.controller(0).sensor('lidar')
lidar_link = w.robot(0).link(int(lidar.getSetting('link')))
lidar_transform = sensing.get_sensor_xform(lidar)
#need to convert lidar_transform to have x pointing forward, y to the left, z up
#Klamp't convention has z pointing forward, x pointing left, y pointing up
klampt_to_ros_lidar = so3.from_matrix([[0,1,0],
                                       [0,0,1],
                                       [1,0,0]])
lidar_transform_ros = (so3.mul(lidar_transform[0],klampt_to_ros_lidar),lidar_transform[1])

rosTfBroadcaster = tf.TransformBroadcaster()

#testing
lidar.kinematicSimulate(w,0.01)
msg = klampt.io.ros.to_SensorMsg(lidar)

#FRAMES = 'manual'
FRAMES = 'auto'

rate = rospy.Rate(5) #hz
if FRAMES == 'manual':
    pub = klampt.io.ros.object_publisher("/base_scan",lidar,{'frame':'base_scan'},queue_size=10)
else:
    pub = klampt.io.ros.object_publisher("/base_scan",lidar,queue_size=10)
while not rospy.is_shutdown():
    t = rospy.Time.now()
    lidar.kinematicSimulate(w,0.01)
    if FRAMES == 'manual':
        #sets up the chain odom -> base_link -> base_scan
        base_transform = lidar_link.getTransform()
        klampt.io.ros.broadcast_tf(rosTfBroadcaster,base_transform,frameprefix="base_link",root="odom",stamp=t)
        klampt.io.ros.broadcast_tf(rosTfBroadcaster,lidar_transform_ros,frameprefix="base_scan",root="base_link",stamp=t)
        klampt.io.ros.broadcast_tf(rosTfBroadcaster,w.robot(0),frameprefix="klampt",root="odom",stamp=t)
    else:
        klampt.io.ros.broadcast_tf(rosTfBroadcaster,w.robot(0),frameprefix="klampt",root="odom",stamp=t)
        klampt.io.ros.broadcast_tf(rosTfBroadcaster,lidar,frameprefix="klampt",stamp=t)
    pub.publish(lidar)
    rate.sleep()

