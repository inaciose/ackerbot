#!/usr/bin/env python3

# Imports
import signal
import sys
import cv2
from csv import writer
import copy
import numpy as np
from numpy.lib.function_base import angle
import rospy
from geometry_msgs.msg._Twist import Twist
from sensor_msgs.msg._Image import Image
from cv_bridge.core import CvBridge
from datetime import datetime
import pandas as pd
import datetime 
from PIL import Image as Image_pil
from std_msgs.msg._Bool import Bool

import pathlib
import os

# Global Variables
global nodeCmd
global angular
global linear
global bridge
global begin_cmd
global begin_img
global img_rbg
global driving_log

global data_path

# Callback Function to receive the nodeCmd values
def message_nodeCmd_ReceivedCallback(message):

    global nodeCmd
    nodeCmd = message.data

# Callback Function to receive the cmd_vel values
def message_Twist_ReceivedCallback(message):

    global angular
    global linear
    global begin_cmd

    angular = float(message.angular.z)
    linear = float(message.linear.x)

    begin_cmd = True

# Callback function to receive image
def message_RGB_ReceivedCallback(message):
    
    global img_rbg
    global bridge
    global begin_img

    img_rbg = bridge.imgmsg_to_cv2(message, "bgr8")

    begin_img = True

    
def signal_handler(sig, frame):
    global driving_log

    global data_path

    print('You pressed Ctrl+C!')
    driving_log.to_csv(data_path + '/driving_log.csv',mode='a',index=False,header=False)
    sys.exit(0)    

def main():

    # Global variables
    global nodeCmd
    global angular
    global linear
    global bridge
    global img_rbg
    global begin_cmd
    global begin_img
    global driving_log

    global data_path

    # Initial Value
    nodeCmd = False
    begin_cmd = False
    begin_img = False
    first_time = True

    # Init Node
    rospy.init_node('ml_write_data', anonymous=False)

    image_raw_topic = rospy.get_param('~image_raw_topic', '/ackermann_vehicle/camera/rgb/image_raw') 
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
    base_folder = rospy.get_param('~base_folder', 'set1') 

    s = str(pathlib.Path(__file__).parent.absolute())
    data_path = s + '/../data/' + base_folder

    # If the path does not exist, create it
    if not os.path.exists(data_path):
        os.makedirs(data_path)
        data_path2 = data_path + '/IMG'
        os.makedirs(data_path2)

    print (data_path)
    print (data_path + '/driving_log.csv')

    # Subscribe topics
    rospy.Subscriber("nodeend", Bool, message_nodeCmd_ReceivedCallback)
    rospy.Subscriber(twist_cmd_topic, Twist, message_Twist_ReceivedCallback)
    rospy.Subscriber(image_raw_topic, Image, message_RGB_ReceivedCallback)

    # Create an object of the CvBridge class
    bridge = CvBridge()

    # Create pandas dataframe
    # driving_log = pd.DataFrame(columns=['Center','Steering'])
    driving_log = pd.DataFrame(columns=['Center','Steering', 'Velocity'])

    # prepare loop
    rate = rospy.Rate(10)
    signal.signal(signal.SIGINT, signal_handler)

    while  True:

        if nodeCmd:
            break
        
        if begin_cmd == False or begin_img == False:
            continue
        
        if linear == 0:
            continue

        curr_time = datetime.datetime.now()
        image_name = str(curr_time.year) + '_' + str(curr_time.month) + '_' + str(curr_time.day)+ '__' + str(curr_time.hour)+ '_' + str(curr_time.minute)+ '_' + str(curr_time.second)+ '__' + str(curr_time.microsecond) + str('.jpg')        
        # add image and angle to the driving_log pandas
        #row  = pd.DataFrame([[image_name,angular]],columns=['Center','Steering'])
        row  = pd.DataFrame([[image_name,angular, linear]],columns=['Center','Steering', 'Velocity'])
        driving_log=driving_log.append(row,ignore_index=True)
        
        # save image
        width = 320
        height = 160
        dim = (width, height)
        img_rbg = cv2.resize(img_rbg, dim, interpolation = cv2.INTER_AREA)
        image_saved = Image_pil.fromarray(img_rbg)
        #image_saved.save('../data/IMG/' + image_name)
        image_saved.save( data_path + '/IMG/' + image_name)
        print('Image Saved')
        
        rate.sleep()


    print('You pressed end')
    driving_log.to_csv(data_path + '/driving_log.csv',mode='a',index=False,header=False)
    sys.exit(0)    

    print ("###################################")
    print ("######## ml_write_data end ########")
    print ("###################################")


if __name__ == '__main__':
    main()

