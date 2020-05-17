#!/usr/bin/python


import rospy

from rospkg import RosPack
from can_msgs.msg import Frame

a = Frame()

class Can_Bridge(object):
    def __init__(self):
        self.msg = Frame()
         