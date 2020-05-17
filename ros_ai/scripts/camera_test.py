#!/usr/bin/python


import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

rospy.init_node("i_see_you")
pub = rospy.Publisher("/camera/rgb/image_raw", Image,queue_size=1)
cap = cv2.VideoCapture(1)

while(not rospy.is_shutdown()):
    ret , frame = cap.read()

    try:
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as e:
        print(e)
    
    pub.publish(msg)