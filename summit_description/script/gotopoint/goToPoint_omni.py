#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math
import numpy as np

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = rospy.get_param('des_pos_yaw')
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def change_state(state):
    global state_
    state_ = state
    print 'State changed to [%s]' % state_

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
def go_to_point(des_pos):
    global yaw_, pub, yaw_precision_, state_

    # Calculate the Errors
    err_yaw = normalize_angle(des_pos.z - yaw_)
    err_pos_y = des_pos.y - position_.y
    err_pos_x = des_pos.x - position_.x

    # Change coordinate frame: odom -> robot
    robot_err_x =   np.cos(yaw_) * err_pos_x + np.sin(yaw_) * err_pos_y
    robot_err_y =   - np.sin(yaw_) * err_pos_x + np.cos(yaw_) * err_pos_y

    twist_msg = Twist()

    # Flag to check whether we finish or not
    there_is_error = False

    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
        there_is_error = True

    if math.fabs(robot_err_x) > dist_precision_:
        twist_msg.linear.x = 0.7 if robot_err_x > 0 else -0.7
        there_is_error = True

    if math.fabs(robot_err_y) > dist_precision_:
        twist_msg.linear.y =  0.7 if robot_err_y > 0 else - 0.7
        there_is_error = True

    pub.publish(twist_msg)

    if not there_is_error:
        print 'Arrived'
        change_state(1)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, active_

    rospy.init_node('go_to_point')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                go_to_point(desired_position_)
            elif state_ == 1:
                done()
            else:
                rospy.logerr('Unknown state!')

        rate.sleep()

if __name__ == '__main__':
    main()