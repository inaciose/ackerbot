#!/usr/bin/env python3

import rospy
from geometry_msgs.msg._Twist import Twist
#from std_msgs.msg import Int16
#import math

# Callback Function to receive the cmd_vel values
def twistInCallback(message):
    global angular_factor
    global linear_default
    global linear_threshold
    global pub_twist_out

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

    pub_twist_out.publish(twist)

def main():

    # Global variables
    global angular_factor
    global linear_default
    global linear_threshold
    global pub_twist_out

    # Init Node
    rospy.init_node('cmd_vel', anonymous=False)

    # Load parameters
    twist_in_topic = rospy.get_param('~twist_in', '/cmd_velin')
    twist_out_topic = rospy.get_param('~twist_out', '/cmd_vel')
    angular_factor = rospy.get_param('~angular_factor', 5)
    linear_default = rospy.get_param('~linear_default', 0.25)
    linear_threshold = rospy.get_param('~linear_threshold', 0.1)
    #pub_dir_topic = rospy.get_param('~pub_dir', '/pub_dir')
    #pub_vel_topic = rospy.get_param('~pub_vel', '/pub_vel')
    #wheelbase = rospy.get_param('~wheelbase', 0.21)
        
    # Subscriber & Publishers   
    rospy.Subscriber(twist_in_topic, Twist, twistInCallback)
    pub_twist_out = rospy.Publisher(twist_out_topic, Twist, queue_size=3)

    rospy.spin()


if __name__ == '__main__':
    main()