#!/usr/bin/env python
import rospy 
import tf

from math import cos,sin
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import Vector3

class Odom_Handler(object):
    
    def __init__(self):
        rospy.init_node("Odomer")
        self.odom_publisher = rospy.Publisher("odom",Odometry,queue_size = 1)
        self.odom_broadcast = TransformBroadcaster()
        self.can_listener = rospy.Subscriber("raw_odom", Vector3, callback=self.handle)

        self.odom_tf = TransformStamped()
        self.odom_msg = Odometry()

        #last_rec -> en son alinan ilerleme miktari 
        self.last_rec = 0

        #hesaplanan x, y ve theta degerleri
        self.x = 0
        self.y = 0
        self.th = 0
    
    def handle(self,data):
        current_time = rospy.Time.now()
        
        dx = (data.x - self.last_rec) * cos(data.z)
        dy = (data.x - self.last_rec) * sin(data.z)

        #dth = data.z * dt #self.z eger hiz olarak alinacaksa
        self.th += data.z
        self.x += dx
        self.y += dy

        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.th)

        self.odom_broadcast.sendTransform((self.x, self.y, 0),
                                        odom_quat,
                                        current_time,
                                        "base_link",
                                        "odom")
        
        self.odom_msg.header.stamp =current_time
        self.odom_msg.header.frame_id = "odom"

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0.
        self.odom_msg.pose.pose.orientation = odom_quat

        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.twist.twist.linear.x = data.x
        self.odom_msg.twist.twist.angular.z = self.th
		
        #publish msg
        self.odom_publisher.publish(self.odom_msg)
        self.last_rec = data.x


if __name__ == "__main__":
	a = Odom_Handler()
	rospy.spin()