#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from move_car import MoveCar
import numpy as np
import math
from summit_description.msg import LineInfo


class LineFollower(object):
    
    def __init__(self):

        self.detect_sub = rospy.Subscriber("line_detect", LineInfo, self.move_callback)
        self.move = MoveCar()
        self.car_twist = Twist()
        self.error = 0
    
    def move_callback(self,data):

        self.error = data.error_x

    def _pid_calculate(self):

        self.car_twist.linear.x = 0.2
        self.car_twist.angular.z = self.error / 300  

        if math.fabs(self.car_twist.angular.z) > 0.10 :
            if self.car_twist.angular.z > 0 :
                self.car_twist.angular.z = 0.10
            else:
                self.car_twist.angular.z = -0.10

    def control(self):

        self.move.move_robot(self.car_twist)
        self._pid_calculate()
        rospy.loginfo("ANGULAR VALUE SENT -->"+str(self.car_twist.angular.z))
        rospy.loginfo("error -->"+str(self.error))


    def stop(self):
        self.car_twist.linear.x = 0.0
        self.car_twist.angular.z = 0.0
        self.move.move_robot(self.car_twist)

def main():

    rospy.init_node("line_following_node", anonymous=True)
    line_follower= LineFollower()
    rate = rospy.Rate(10)

    def shutdown():
        print("shutdown!")
        line_follower.stop()

    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        line_follower.control()
        rate.sleep()

if __name__ == '__main__':
    main()
