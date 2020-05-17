#!/usr/bin/env python
import rospy 
import tf

from math import cos,sin
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import Vector3


#Vector3.x -> x linear velocity
#Vector3.y -> y linear velocity which is always 0
#Vector3.z -> z steer angle 


###########################
#i might remove tf broadcaster from this node
#because mustafa coded a tf broadcaster i might
#use it 
###########################
class Odom_Handler():
	"""
		purpose of this class is estimating odom 
		and publish it
	"""
	def __init__(self):
		rospy.init_node("Odomer")
		self.odom_publisher = rospy.Publisher("odom",Odometry,queue_size = 50)
		self.odom_broadcast = TransformBroadcaster()
		self.can_listener = rospy.Subscriber("raw_odom",Vector3,callback=self.handler)

		self.odom_tf = TransformStamped()
		self.odom_msg = Odometry()
		#self.rate = rospy.Rate(5)
		self.x = 0
		self.y = 0
		self.th = 0
		# 	self.vx = 0
		# there is no need for vy in our car
		self.z = 0 
		#changes 
		#current_time = rospy.Time.now()
		self.dt = 0 #dt = (current_time - last_time).toSec()
		self.dx = 0 #delta_x = (vx * cos(th) - vy * sin(th)) * dt
		self.dy = 0 #delta_y = (vx * sin(th) + vy * cos(th)) * dt
		self.dth = 0 #delta_th = vth * dt
		self.last_time = rospy.Time.now()

	def handler(self,data):
		current_time = rospy.Time.now()
		dt = (current_time - self.last_time).to_sec()
		dx = data.x * cos(data.z) * dt
		dy = data.x * sin(data.z) * dt
		dth = data.z * dt #self.z eger hiz olarak alinmiyorsa bu gereksiz
		
		self.x += dx
		self.y += dy
		self.th += dth # bu da gereksiz olur
		
		#tf frame info
		odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)
		"""
		self.odom_tf.header.stamp = current_time
		self.odom_tf.header.frame_id = "odom"
		self.odom_tf.child_frame_id = "base_link"

		self.odom_tf.transform.translation.x = self.x
		self.odom_tf.transform.translation.y = self.y
		self.odom_tf.transform.translation.z = 0.0
		self.odom_tf.transform.rotation = odom_quat
		self.odom_publisher.publish(self.odom_tf)
		"""
		#broadcast
		#check i think there is no problem
		self.odom_broadcast.sendTransform((self.x, self.y, 0),
										odom_quat,
										current_time,
										"base_link",
										"odom")
		
		#ros Odometry msg info
		self.last_time = current_time
		


if __name__ == "__main__":
	Odom_Handler()
	rospy.spin()