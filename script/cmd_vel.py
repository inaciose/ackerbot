#!/usr/bin/env python3

import rospy
from geometry_msgs.msg._Twist import Twist
from std_msgs.msg import Int16
import math

def map(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

# Callback Function to receive the cmd_vel values
def twistInCallback(message):
    global angular_factor
    global linear_default
    global linear_threshold
    global pub_twist_out
    global pub_dir_out
    global pub_vel_out
    global wheelbase

    angular = float(message.angular.z)
    linear = float(message.linear.x)

    angular = angular * angular_factor
    if linear > linear_threshold:
        linear = linear_default
    else:
        linear = 0

    # Publish twist
    twist = Twist()
    twist.linear.x = linear
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angular

    #pub_twist_out.publish(twist)


    # Publish pub_dir & pub_vel
    if linear == 0 or angular == 0:
        steering_angle = 0
    else:
        radius = linear / angular
        steering_angle = math.atan(wheelbase / radius)

    steering_angle = -steering_angle

    # scale it to use it with the servo (value between 0 and 180)
    servo1Angle = map(steering_angle * 100, -156, 156, 50, 130)

    if servo1Angle < 60:
        servo1Angle = 60
    if servo1Angle > 120: 
        servo1Angle = 120

    leftSpeed = linear * 2 - ((angular * (wheelbase * 0.5)) / 2 + linear)
    

    servo1 = Int16()
    servo1.data = int(servo1Angle)
    lspeed = Int16()
    lspeed.data = int(leftSpeed * 1000)

    pub_dir_out.publish(servo1)
    pub_vel_out.publish(lspeed)

def main():

    # Global variables
    #global angular
    global angular_factor
    #global linear
    global linear_default
    global linear_threshold
    global pub_twist_out
    global pub_dir_out
    global pub_vel_out
    global wheelbase

    # Init Node
    rospy.init_node('cmd_vel', anonymous=False)

    # Load parameters
    twist_in_topic = rospy.get_param('~twist_in', '/cmd_velin')
    twist_out_topic = rospy.get_param('~twist_out', '/cmd_vel')
    angular_factor = rospy.get_param('~angular_factor', 5)
    linear_default = rospy.get_param('~linear_default', 0.25)
    linear_threshold = rospy.get_param('~linear_threshold', 0.1)
    pub_dir_topic = rospy.get_param('~pub_dir', '/pub_dir')
    pub_vel_topic = rospy.get_param('~pub_vel', '/pub_vel')
    wheelbase = rospy.get_param('~wheelbase', 0.21)
        
    # Subscriber & Publishers   
    rospy.Subscriber(twist_in_topic, Twist, twistInCallback)
    
    # twist mode
    pub_twist_out = rospy.Publisher(twist_out_topic, Twist, queue_size=3)
 
    # vel dir mode
    pub_dir_out = rospy.Publisher(pub_dir_topic, Int16, queue_size=3)
    pub_vel_out = rospy.Publisher(pub_vel_topic, Int16, queue_size=3)

    rospy.spin()


if __name__ == '__main__':
    main()