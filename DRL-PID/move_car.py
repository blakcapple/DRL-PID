#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy

class MoveCar(object):
    def __init__(self):

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        # self.cmd_vel_subs = rospy.Subscriber('/cmd_vel',Twist,self.cmdvel_callback)
        # self.last_cmdvel_command = Twist()
        # self._cmd_vel_pub_rate = rospy.Rate(10)

    # def cmdvel_callback(self,msg):
    #     self.last_cmdvel_command = msg
    
    # def compare_twist_commands(self,twist1,twist2):
    #     LX = twist1.linear.x == twist2.linear.x
    #     LY = twist1.linear.y == twist2.linear.y
    #     LZ = twist1.linear.z == twist2.linear.z
    #     AX = twist1.angular.x == twist2.angular.x
    #     AY = twist1.angular.y == twist2.angular.y
    #     AZ = twist1.angular.z == twist2.angular.z
    #     equal = LX and LY and LZ and AX and AY and AZ
    #     if not equal:
    #         rospy.logwarn("The cunrrent twist is not the same as the one sent;Resending")
    #     return equal

    def move_robot(self, twist_object):
        self.cmd_vel_pub.publish(twist_object)
        # current_equal_to_new = self.compare_twist_commands(twist1=self.last_cmdvel_command, 
        #                                                     twist2 = twist_object)

    def clean_class(self):
        twist_object = Twist()
        twist_object.linear.x = 0.0
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_dectected = True
    
def main():
    rospy.init_node('move_robot_node')

    move_object = Movecar()
    twist_object = Twist()
    twist_object.angular.z = 0.5
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdown_dectected():
        move_object.clean_class()
        rospy.loginfo("shutdown time!")

    rospy.on_shutdown(shutdown_dectected)

    while not ctrl_c:
        move_object.move_robot(twist_object)
        rate.sleep()

if __name__=='__main__':
        main()
     