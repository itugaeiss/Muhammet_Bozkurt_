#!/usr/bin/env python
import rospy
import threading

from std_msgs.msg import Float32
from pynput.keyboard import Key, Listener,GlobalHotKeys

msg = Float32()
msg.data = 50.0
rospy.init_node("tester")
pub = rospy.Publisher("tester",Float32,queue_size=1)
rate = rospy.Rate(5)


def on_press(key):
    if(key ==Key.up):
        if(msg.data<=550):
            msg.data+=5
        print("faster")
    elif(key == Key.down):
        if(msg.data>=15):
            msg.data-=5
        print("slower")
    elif(key == Key.esc):
        return False

l = Listener(on_press=on_press)
l.start()

while(not rospy.is_shutdown()):
    pub.publish(msg)
    rate.sleep()