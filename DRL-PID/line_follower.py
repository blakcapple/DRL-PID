#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from move_car import MoveCar
import numpy as np
import math
from summit_description.msg import EnvInfo
import time
import matplotlib.pyplot as plt
from std_srvs.srv import Empty 
from nav_msgs.msg import Odometry

#max v_x=1;
class LineFollower(object):
    
    def __init__(self):

        self.node = rospy.init_node('line_following_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.detect_sub = rospy.Subscriber("env_feedback", EnvInfo, \
                                                            self.move_callback)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pos_sub = rospy.Subscriber('/odom', Odometry, self.pos_callback)
        self.move = MoveCar()
        self.car_twist = Twist()
        self.pose_list=[[0,0],]
        self.distance = 0
        self.time = time.time()
        # infomation to control car
        self.error = list(np.zeros(5))
        self.error_k = list(np.zeros(5))
        self.angular_list=[0,0,0]
        self.last_error = 0 # error made by last action
        self.v_x =0
        self.w_z =0
        self.cx = np.zeros(5)
        self.cy = np.zeros(5)
        # the action take
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.kp2 = 0
        self.ki2 = 0
        self.kd = 0
        # tradintional pid gain
        self._kp = 4
        self._kd = 0.2
        self._ki = 0.05 #0.005*3
        self._kp2 = 2
        self._kd2 = 0.2
        self._ki2 = 0
        # folloing will return to agent
        self.reward = 0
        self.state = np.zeros(13)
        #shape
        self.action_space = np.zeros(6)
        self.state_space = np.zeros(13)
        self.max_action = 10
        self.time_step = 0
        self.errorx_list = [] #error list
        self.reset_flag = True
        self.success_record = [] # record the success time

        self.done = False
        self.failed_flag = False
        self.success_flag = False
        self.sum_errork=0
        self.sum_errorx=0

    def pos_callback(self, data):
        """linear and angular"""
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        dist = (x-self.pose_list[-1][0])**2+(y-self.pose_list[-1][1])**2
        self.distance += math.sqrt(dist)
        # if self.distance >= goal:
        #     self.detect_msg.success_flag = 1
        #     print("success!")

        self.pose_list.append([x, y])

                          
        self.v_x = data.twist.twist.linear.x
        self.w_z = data.twist.twist.angular.z

        if abs(x) <= 0.5 and abs(y) <= 0.5 and \
            (time.time()-self.time) > 20:
            self.success_flag = True

    def reset(self):
        print('mean_v:', self.distance/(time.time()-self.time))
        self.reset_world()
        if self.failed_flag:
            self.success_record.append(0)
            print("fail!")
        if self.success_flag:
            self.success_record.append(1)
            print("success!")
        self.error = list(np.zeros(5))
        self.error_k = list(np.zeros(5))
        self.angular_list=[0,0,0]
        self.last_error = 0
        self.v_x = 0
        self.w_z = 0
        self.reward = 0
        self.state = np.zeros(13)
        self.time_step = 0
        self.stop()
        time.sleep(3)    
        self.time = time.time()
        self.errorx_list=[]
        self.done = False
        self.failed_flag = False
        self.success_flag = False
        self.pose_list = [[0,0],]
        self.distance = 0
        self.last_distance =0
        self.sum_errork=0
        self.sum_errorx=0

        return self.state

    def move_callback(self,data):

        self.error.pop()
        self.error.insert(0, data.error_x)
        self.error_k.pop()
        self.error_k.insert(0, data.error_k)
        self.cx = np.array(data.cx)
        self.cy = np.array(data.cy)
        self.failed_flag = data.failed_flag
        self.done = self.success_flag or self.failed_flag 
        
        self.errorx_list.append(self.error[0])

    def _reward_calculate(self):

        var_e = np.var(self.error)

        reward = 2*(self.distance - self.last_distance) - 0.1 - 10*var_e

        if self.done :
            if self.success_flag:
                reward = 5
            else:
                reward = -5

        self.last_error = self.error[0]
        self.last_distance = self.distance
        return reward

    def _pid2_calculate(self):

        er1_k = self.error_k[0]
        er2_k = self.error_k[1]
        er3_k = self.error_k[2]
        add_wz_2 = self._kp2 * (er1_k - er2_k) + self._ki2 * er1_k + \
                    self._kd2 * (er1_k - 2 * er2_k + er3_k)
        return add_wz_2

    def _pid_calculate(self):

        er1 = self.error[0]
        er2 = self.error[1]
        er3 = self.error[2]

        self.car_twist.linear.x = 2-abs(self.error[0])*0.5

        add_wz = -(self._kp * (er1 - er2) + self._ki * er1 + \
                    self._kd * (er1 - 2 * er2 + er3))
        angular_z = self.car_twist.angular.z
        angular_z += (add_wz+0.5*self._pid2_calculate())
        # angular_z = max(-np.pi, min(np.pi, angular_z)) # clip(-0.1, 0.1)
        self.car_twist.angular.z = self.car_twist.angular.z*0.1+0.9*angular_z

    # def _pid2_rl(self):

    #     er1_k = self.error_k[0]
    #     er2_k = self.error_k[1]
    #     er3_k = self.error_k[2]
    #     self.sum_errork +=er1_k
    #     wz_2 = self.kp2*er1_k + self.kd2*(er1_k-er2_k) 
    #     return wz_2

    # def _pid_rl(self):
        
    #     er1 = self.error[0]
    #     er2 = self.error[1]
    #     er3 = self.error[2]
    #     self.sum_errorx +=er1
    #     self.car_twist.linear.x = 2-abs(self.error[0])*0.5

    #     wz_1 = -(self.kp*er1+self.kd*(er1-er2))
    #     angular_z = wz_1 + self._pid2_rl()*0.5
    #     angular_z = max(-np.pi, min(np.pi, angular_z)) # clip(-1, 1)
    #     self.car_twist.angular.z = self.car_twist.angular.z*0.1+0.9*angular_z
    #     self.angular_list.pop()
    #     self.angular_list.insert(0,self.car_twist.angular.z)
    
    def _pid2_rl(self):

        er1_k = self.error_k[0]
        er2_k = self.error_k[1]
        er3_k = self.error_k[2]
        add_wz_2 = self.kp2 * (er1_k - er2_k) + self.ki2 * er1_k + \
                    self.kd2 * (er1_k - 2 * er2_k + er3_k)
        return add_wz_2

    def _pid_rl(self):

        er1 = self.error[0]
        er2 = self.error[1]
        er3 = self.error[2]

        self.car_twist.linear.x = 2-abs(self.error[0])*0.5

        add_wz = -(self.kp * (er1 - er2) + self.ki * er1 + \
                    self.kd * (er1 - 2 * er2 + er3))
        angular_z = self.car_twist.angular.z
        angular_z += (add_wz+0.5*self._pid2_rl())
        angular_z = max(-np.pi, min(np.pi, angular_z)) # clip(-0.1, 0.1)
        self.car_twist.angular.z = self.car_twist.angular.z*0.1+0.9*angular_z


    def control(self,sac_pid=True):
        if not sac_pid:
            self._pid_calculate()
        else:
            self._pid_rl()
        # rospy.loginfo("ANGULAR VALUE SENT -->"+str(self.car_twist.angular.z))
        # rospy.loginfo("error -->"+str(self.error[0]))
        self.move.move_robot(self.car_twist)

    def feedback(self):
        # state and reward
        self.state = np.append(self.cx,self.cy)
        # self.state1 = np.append(self.error,self.error_k)
        # self.state = np.append(self.state, self.state1)
        state2 = [self.v_x,self.w_z,self.error_k[0], self.distance/45.0]
        self.state = np.append(self.state, state2)
        self.reward = self._reward_calculate()

        return self.state, self.reward, self.done

    def update_pid(self,kp,kd,kp2,kd2,ki=None,ki2=None):
        if ki and ki2:
            self.kp = kp
            self.kd = kd
            self.ki = ki
            self.kp2 = kp2
            self.ki2 = ki2
            self.kd2 = kd2
        else:
            self.kp = kp
            self.kd = kd
            self.kp2 = kp2
            self.kd2 = kd2

        # print('P1:%.1f'%self.kp,' D1: %.1f'%self.kd,' I1: %.1f'%self.ki)
        # print('P2:%.1f'%self.kp2,' D2: %.1f'%self.kd2,' I2: %.1f'%self.ki2)

    def stop(self):
        self.car_twist.linear.x = 0.0
        self.car_twist.angular.z = 0.0
        self.move.move_robot(self.car_twist)

def main():

    rospy.init_node("line_following_node", anonymous=True)
    line_follower= LineFollower()
    rate = rospy.Rate(10)
    line_follower.reset()

    def shutdown():
        print("shutdown!")
        line_follower.stop()

    rospy.on_shutdown(shutdown)
    while not rospy.is_shutdown():
        line_follower.control(sac_pid=False)
        if line_follower.done:
            total_time = time.time()-line_follower.time
            print(line_follower.distance/(total_time))
            line_follower.reset()
        rate.sleep()




if __name__ == '__main__':
    main()
