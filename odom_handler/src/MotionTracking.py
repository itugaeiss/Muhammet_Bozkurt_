#!/usr/bin/env python

from optparse import OptionParser
import roslib, rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

def draw_field(gray_image, points, flow):
    '''
    gray_image: opencv gray image, e.g. shape = (width, height)
    points: points at which optic flow is tracked, e.g. shape = (npoints, 1, 2)
    flow: optic flow field, should be same shape as points
    '''
    og_img = gray_image.copy()
    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0,0,255] # bgr colorspace
    linewidth = 1
    for i, point in enumerate(points):
        x = point[0,0]
        y = point[0,1]
        vx = flow[i][0,0]
        vy = flow[i][0,1]
        cv2.line(color_img, (x,y), (x+vx, y+vy), color_red, linewidth) # draw a red line from the point with vector = [vx, vy]
        cv2.circle(color_img,(x+vx, y+vy),3,color_red,-1) #draw a circle at the point

    cv2.imshow('optic_flow_field',color_img)
    cv2.waitKey(1)

################################################################################


################################################################################

class Optic_Flow_Calculator:
    def __init__(self, topic):
        self.image_source = "/zed/right/image_raw"

        self.bridge = CvBridge()
        self.prev_image = None
        self.last_time = 0

        # Lucas Kanade Optic Flow parameters change as desired
        self.lk_params = dict(winSize  = (150,150),maxLevel = 2,criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 0.03))

        self.image_sub = rospy.Subscriber(self.image_source,Image,self.image_callback)

    def image_callback(self,image):
        try:# get image, and convert to single channel gray image
            curr_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8")
            curr_image = curr_image[0: 280, 0:900]
            curr_image = cv2.resize(curr_image, (1800,1600))

            # If this is the first loop, initializ the info
            if self.prev_image is None:
                self.prev_image = curr_image
                self.points_to_track = cv2.goodFeaturesToTrack(curr_image,25,0.01,10)
                return # skip the rest of this loop

            self.points_to_track = cv2.goodFeaturesToTrack(curr_image,25,0.01,10)

            new_position_of_tracked_points, status, error = cv2.calcOpticalFlowPyrLK(self.prev_image, curr_image, self.points_to_track, None, **self.lk_params)

            flow = new_position_of_tracked_points - self.points_to_track

            # draw the arrow
            draw_field(curr_image, self.points_to_track, flow)

            # save current image and time for next loop
            self.prev_image = curr_image

        except CvBridgeError, e:
            print e

    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down"
            cv2.destroyAllWindows()

################################################################################

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default='',
                        help="ros topic with Float32 message for velocity control")
    (options, args) = parser.parse_args()

    rospy.init_node('optic_flow_calculator', anonymous=True)
    optic_flow_calculator = Optic_Flow_Calculator(options.topic)
    optic_flow_calculator.main()
