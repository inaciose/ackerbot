#!/usr/bin/env python3

# Imports
import argparse
import cv2
from csv import writer
import copy
import numpy as np
import rospy
from geometry_msgs.msg._Twist import Twist
from sensor_msgs.msg._Image import Image
from cv_bridge.core import CvBridge
from datetime import datetime
from tensorflow.keras.models import load_model
import pathlib
import os
import string
import time
from std_msgs.msg._Bool import Bool

global nodeCmd
global img_rbg
global bridge
global begin_img
global pub
global twist_cmd_topic

def preProcess(img):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img,  (3, 3), 0)
    img = cv2.resize(img, (200, 66))
    img = img/255
    return img

def message_nodeCmd_ReceivedCallback(message):
    global nodeCmd
    nodeCmd = message.data

def message_RGB_ReceivedCallback(message):
    global img_rbg
    global bridge
    global begin_img

    img_rbg = bridge.imgmsg_to_cv2(message, "bgr8")
    begin_img = True

def shutdown_hook():
    print("###################")
    print("## shutting down ##")
    print("###################")

def main():

    # Global variables
    global nodeCmd
    global img_rbg
    global bridge
    global begin_img
    global pub
    global twist_cmd_topic

    nodeCmd = False
    begin_img = False
    twist = Twist()

    # Init Node
    rospy.init_node('ml_driving', anonymous=False)

    # set ros parameters
    image_raw_topic = rospy.get_param('~image_raw_topic', '/ackermann_vehicle/camera/rgb/image_raw') 
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    twist_linear_x = rospy.get_param('~twist_linear_x', 0.25)
    modelname = rospy.get_param('~modelname', 'model1.h5')

    # load model
    s = str(pathlib.Path(__file__).parent.absolute())
    path = s + '/../../model/' + modelname
    print (path)
    model = load_model(path)

    # Subscribe and publish topics
    rospy.Subscriber("nodeend", Bool, message_nodeCmd_ReceivedCallback)
    rospy.Subscriber(image_raw_topic, Image, message_RGB_ReceivedCallback)
    pub = rospy.Publisher(twist_cmd_topic, Twist, queue_size=5)

    # Create an object of the CvBridge class
    bridge = CvBridge()

    # prepare main loop
    rate = rospy.Rate(10)
    rospy.on_shutdown(shutdown_hook)

    # start main loop
    while not rospy.is_shutdown():

        if nodeCmd:
            break
        
        if begin_img == False:
            continue

        resized_ = preProcess(img_rbg)

        # debug
        #cv2.imshow('Robot View Processed', resized_)
        #cv2.imshow('Robot View', img_rbg)
        #cv2.waitKey(1)

        # Predict angle
        image = np.array([resized_])
        steering = float(model.predict(image))
        angle = steering

        # Send twist
        twist.linear.x = twist_linear_x
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angle

        pub.publish(twist)

        rate.sleep()

    print ("###################################")
    print ("######### ml_driving stop #########")
    print ("###################################")
    # Send twist
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    time.sleep(1)
    pub.publish(twist)

if __name__ == '__main__':
    main()

