#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy

class MoveCar(object):
    def __init__(self):

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)


    def clean_class(self):
        twist_object = Twist()
        twist_object.linear.x = 0.0
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_dectected = True
     