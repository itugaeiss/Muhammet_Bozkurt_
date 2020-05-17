#!/usr/bin/python

import rospy
import rospkg
import cv2
import json
import numpy as np

from keras import models
from keras.models import Model
from keras.layers import Conv2D
from keras.layers.merge import add
from keras.layers import Dropout, Flatten, Dense, Input, Lambda, SpatialDropout2D
from keras.layers.normalization import BatchNormalization
from keras.layers.advanced_activations import ELU
from keras.models import load_model,model_from_json,Sequential
from geometry_msgs.msg import Vector3
from keras.constraints import max_norm

from keras.backend import clear_session

import tensorflow as tf
global graph,model

graph = tf.get_default_graph()

"""
sending necessary data via vector3 message type 

Vector3.x --> speed

Vector3.y --> steer angle

Vector3.z --> throttle

"/current_CAN_data"(Vector3) topic will be used for getting current CAN data

"/commands"(Vector3) will be used for sending necessary data to drive car

"/camera/center" will be used for getting images

"""

from sensor_msgs.msg import Image as Img
from cv_bridge import CvBridge

MAX_SPEED = 25
MIN_SPEED = 10

IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS = 160, 320, 3


class SteerNode():
    def __init__(self):
        path = rospkg.RosPack().get_path("ros_ai")
        self.bridge = CvBridge()
        #maybe one day
        #self.sub_can_data = rospy.Subscriber("/current_CAN_data", Vector3, self.callback_curr_data)
        #####################################################################3
        #print("{}/scripts/steer.h5".format(path))
        #self.model = load_model("{}/scripts/steer.model".format(path),compile=False)#nvidia end-to-end self driving car paper
        # load the model:
        clear_session()
        #self.model = self.build_model()
        self.model = load_model("{}/scripts/steer.model".format(path),compile=False)

        #self.model.load_weights("{}/scripts/steer.h5".format(path))
        #self.model = load_model("{}/scripts/end-to-end.model".format(path))


        if(self.model is None):
            print("---------------------------------model")
        else:
            print("model---------------------------------")

        self.command = Vector3()
        
        self.command.x = 0
        self.command.y = 0
        self.command.z = 0
        self.speed_limit = MAX_SPEED
    
        self.pub= rospy.Publisher("/commands", Vector3,queue_size=1)
        self.sub_img = rospy.Subscriber("/camera/rgb/image_raw", Img, self.callback_steer)
    """
    original version
    def callback_steer(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        img = self.preprocess(img)
        self.command.y = self.model.predict(img)
        self.pub.publish(self.command)
    
    
    def callback_curr_data(self, data):
        if(data.speed > self.speed_limit):
            self.speed_limit = MIN_SPEED
        else:
            self.speed_limit = MAX_SPEED
        self.command.z = 1.0 - self.command.y**2 - (data.speed/self.speed_limit)**2
        self.command.x = self.speed_limit
    """
    #for testing
    def callback_steer (self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        img = self.preprocess(img)
        img = np.array([img])
        print("after preprocess")
        clear_session()
        self.command.y = self.model.predict(img)
        if(self.command.x > self.speed_limit):
            self.speed_limit = MIN_SPEED
        else:
            self.speed_limit = MAX_SPEED
        self.command.z = 1.0 - self.command.y**2 - (self.command.x/self.speed_limit)**2
        self.pub.publish(self.command)

    def preprocess(self, image):
        image = image[60:-25, :, :] # change depending on real cam
        image = cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT), cv2.INTER_AREA)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
        return image

    def build_model(self):
        model=Sequential()
        model.add(Lambda(lambda x: x/127.5-1.0,input_shape=(160,320,3)))
        #ELU(Exponential linear unit) function takes care of the Vanishing gradient problem.
        model.add(Conv2D(24,5,5,activation="elu",subsample=(2,2)))
        model.add(Conv2D(36,5,5,activation="elu",subsample=(2,2)))
        model.add(Conv2D(48,5,5,activation="elu",subsample=(2,2)))
        model.add(Conv2D(64,5,5,activation="elu"))
        model.add(Conv2D(64,5,5,activation="elu"))
        model.add(Dropout(0.5))
        model.add(Flatten())
        model.add(Dense(100,activation="elu"))
        model.add(Dense(50,activation="elu"))
        model.add(Dense(10,activation="elu"))
        model.add(Dense(1))

        model.summary()
        return model
    
def main():

    print("basladi")
    rospy.init_node("steer_node",  anonymous = True)
    print("init_node sonrasi")
    SteerNode()
    print("steerNode sonrasi")

    print("-------------------------------------------------")
    print("-------------------------------------------------")
    print("-------------------------------------------------")
    print("-------------------------------------------------")
    try:
        rospy.spin()
    except Exception as e:
        print("Exception:\n", e,"\n")
    print("bitti")

if __name__ == "__main__":
    main()



"""

"""