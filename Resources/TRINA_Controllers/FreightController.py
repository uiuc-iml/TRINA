import roslib
import rospy
import time
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys

#Constants
SAFETY_FACTOR = 0.8
MAX_LINEAR_VEL = 2.1054 * SAFETY_FACTOR 
MIN_LINEAR_VEL = -2.1054 * SAFETY_FACTOR
MAX_ANGULAR_VEL = 4.0 * SAFETY_FACTOR
MIN_ANGULAR_VEL = -4.0 * SAFETY_FACTOR
MAX_LINEAR_ACC = 1.0
MIN_LINEAR_ACC = -1.0
MAX_ANGULAR_ACC = 2.0
MIN_ANGULAR_ACC = -2.0


class FreightController:
	def __init__(self):
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		self.sub = rospy.Subscriber('odom',Odometry,self.callback)

		self.dt = 0.01
		self.cmdLinearVel = 0.0 #x direction
		self.cmdAngularVel = 0.0 #z direction
		self.currentLinearVel = 0.0
		self.currentAngularVel = 0.0
		self.lastStateTime = 0.0
		self.exit = False

	def start(self):
		rospy.init_node('FreightController')
		controlThread = threading.Thread(target = self.controlLoop)
		controlThread.start()

	def controlLoop(self):		
		now = rospy.get_rostime()
		self.lastStateTime = float(now.secs) + 1e-9*float(now.nsecs) #get the ros time for each robot state msg
		rate = rospy.Rate(1.0/self.dt)
		while not rospy.is_shutdown() and (not self.exit):
			
			twist = Twist()
			twist.linear.x = self.cmdLinearVel
			twist.linear.y = 0.0
			twist.linear.z = 0.0
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = self.cmdAngularVel
			self.pub.publish(twist)
			
			#print 'flag1'
			rate.sleep()
	def callback(self,data):
		self.lastStateTime = float(data.header.stamp.secs)+1e-9*float(data.header.stamp.nsecs)
		self.currentLinearVel = data.twist.twist.linear.x
		self.currentAngularVel = data.twist.twist.angular.z
		#print 'flag2',
		
	def setVelocity(self,velocity):
		now = rospy.get_rostime()
		currentTime = float(now.secs) + 1e-9*float(now.nsecs)
		velocity = self.limitVelocity(velocity)

		acc = [0.0,0.0]
		delT = currentTime-self.lastStateTime
		acc[0] = (velocity[0] - self.currentLinearVel)/delT
		acc[1] = (velocity[1] - self.currentAngularVel)/delT
		#print velocity,acc,self.currentLinearVel,self.currentAngularVel,delT
		#print currentTime, self.lastStateTime

		if acc[0] > MAX_LINEAR_ACC:
			print "Commanded Velocity Exceeding Acceleration Limits, limiting.."
			velocity[0] = self.currentLinearVel+delT*MAX_LINEAR_ACC
		elif acc[0] < MIN_LINEAR_ACC:
			print "Commanded Velocity Exceeding Acceleration Limits, limiting.."
			velocity[0] = self.currentLinearVel+delT*MAX_LINEAR_ACC


		if acc[1] > MAX_ANGULAR_ACC:
			print "Commanded Velocity Exceeding Acceleration Limits, limiting.."
			velocity[1] = self.currentLinearVel+delT*MAX_ANGULAR_ACC
		elif acc[1] < MIN_ANGULAR_ACC:
			print "Commanded Velocity Exceeding Acceleration Limits, limiting.."
			velocity[1] = self.currentLinearVel+delT*MAX_ANGULAR_ACC

		velocity = self.limitVelocity(velocity)
		self.cmdLinearVel = velocity[0]
		self.cmdAngularVel = velocity[1]



	def limitVelocity(self,velocity):
		if velocity[0] > MAX_LINEAR_VEL:
			print "Commanded Velocity Exceeding Velocity Limits, limiting.."
			velocity[0] = MAX_LINEAR_VEL
		elif velocity[0] < MIN_LINEAR_VEL:
			print "Commanded Velocity Exceeding Velocity Limits, limiting.."
			velocity[0] = MIN_LINEAR_VEL

		if velocity[1] > MAX_ANGULAR_VEL:
			print "Commanded Velocity Exceeding Velocity Limits, limiting.."
			velocity[1] = MAX_ANGULAR_VEL
		elif velocity[1] < MIN_ANGULAR_VEL:
			print "Commanded Velocity Exceeding Velocity Limits, limiting.."
			velocity[1] = MIN_ANGULAR_VEL

		return velocity


		#velocity is a tuple 
		self.cmdLinearVel = velocity[0]
		self.cmdAngularVel = velocity[1]

	def stop(self):
		self.exit = True

if __name__ == "__main__":
	freight = FreightController()
	freight.start()
	startTime = time.time()
	while (time.time()-startTime) < 5.0:
		t = time.time() - startTime
		if t < 2.5:
			freight.setVelocity([0.1*t,0.0])
		else:
			freight.setVelocity([0.5-0.1*t,0.0])
		time.sleep(0.01)
	freight.setVelocity([0.0,0.0])
	freight.stop()