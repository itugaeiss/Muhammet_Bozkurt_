#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import geometry_msgs.msg
#suscribe to the odometry
#define/calculate frame
#transform

def odom_tf(msg):
    broadcast = tf2_ros.TransformBroadcaster()
    transform = geometry_msgs.msg.TransformStamped()# stamped or not
    #whats the child frame steering wheel?
    # Parent frame  is base_link or lidar
    # Stamped info
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "Odom2tf" #NOT VERY CERTAIN
    transform.child_frame_id = "tf"#NOT VERY CERTAIN
    # Loacation info
    transform.transform.translation.x = msg.pose.pose.position.x
    transform.transform.translation.y = msg.pose.pose.position.y
    transform.transform.translation.z = 0.0
    # quaternion transformation
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.w)
    # Orientation info
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]
    print transform.transform.rotation.z
    # Publish info
    broadcast.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('Odometry_TF_TransformBroadcaster', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_tf)#SUSCRIBE::Odom data over CAN CURRENTLY::turtlebot3
    print("AADASDA")
    rospy.spin()

why
what iv done

