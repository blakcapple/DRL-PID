#! /usr/bin/env python
import rospy
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from summit_description.msg import EnvInfo
from std_srvs.srv import Empty 
import numpy as np
from nav_msgs.msg import Odometry
# import tf.transformations
from  summit_description.srv import StartUp, StartUpResponse
import time
import math

global img_height,img_width,goal
img_height = 72
img_width = 128
goal = 10 # target distance


class EnvFeedback:
    
    def __init__(self):

        self.bridge = CvBridge()

        self.active = False
        
        self.image_sub = rospy.Subscriber("iRobot/camera/image_raw", Image,
                                self.camera_callback)
        
        self.detect_pub = rospy.Publisher("env_feedback", EnvInfo, queue_size=10)

        self.detect_msg = EnvInfo()

        self.pub_rate = rospy.Rate(10)

        self.step = 0

        self.active = False

        self.pose_list=[[0,0],]

        self.distance = 0

        self.time = time.time()

        self.detect_msg.success_flag = 0
        
        self.detect_msg.failed_flag = 0

        self.center_x = np.zeros(5)
        self.center_y = np.zeros(5)
        
    def pre_cal(self, h):
        h_list = [0]
        for i in range(5):
            h_list.append((i+1)*h/5)
        return h_list 

        #calculate distance
    def dis_cal(self):
        length = len(self.pose_list)
        dis = 0
        for i in range(length-1):
            a = self.pose_list[i+1][0]-self.pose_list[i+1][0]
            b = self.pose_list[i+1][1]-self.pose_list[i+1][1]
            dis +=math.sqrt((a**2+b**2))
        return dis
        

    def camera_callback(self, data):
        if not self.active:
            return
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        except CvBridgeError as e :
            print(e)

        cv_image = cv2.resize(cv_image, (img_width, img_height), \
                                                interpolation=cv2.INTER_CUBIC)
        # cv2.imshow("origin_image", cv_image)
        # ostu threshold
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        threshold = self.ostu(gray)
        threshold = min(100,threshold)
        mask = cv2.inRange(gray, 0, threshold)

        left_list = self.search_leftbound(mask)
        right_list = self.search_rightbound(mask)
        if right_list==-1 or left_list==-1:
            self.detect_msg.failed_flag = True
        else:
            self.detect_msg.failed_flag = False
        if not self.detect_msg.failed_flag:

            img_len = min(len(left_list),len(right_list))-1
            for i in range(5):
                self.center_x[i] = (right_list[img_len*i/4] + left_list[img_len*i/4])/2
                self.center_y[i] = img_len/4.0*i
            self.center_x = np.flip(self.center_x, 0)
            for i in range(5):
                cv2.circle(mask, (int(self.center_x[i]), int(self.center_y[i])), 2, (0,0,0), -1)
            k = self.cur_fit(self.center_y, self.center_x)
            self.detect_msg.error_k = 0.9*k + 0.1*self.detect_msg.error_k
            self.detect_msg.cx = (self.center_x-np.ones(5)*img_width/2.0)/(img_width/2.0)
            self.detect_msg.cy = self.center_y/img_height
            error = self.detect_msg.cx[3]
            self.detect_msg.error_x = 0.9*error + 0.1*self.detect_msg.error_x
            # print(self.detect_msg.error_k)
            
            cv2.imshow("mask",mask)
            cv2.waitKey(1)

    def medium_filter(self,image):
        img = image[:]
        for i in range(1,image.shape[0]-1):
            for j in range(1,image.shape[1]-1):
                temp = np.zeros((3,3))
                for x in range(3):
                    for y in range(3):
                        temp[x,y] = image[i+x-1,j+y-1]
                img[i,j] = np.median(temp)
        return img



    def cur_fit(self,x,y):
        """fitting function"""
        
        x = np.array(x)
        y = np.array(y)
        avg_x = float(x.sum() / x.shape[0])
        avg_y = float(y.sum() / y.shape[0])
        xy = x*y
        avg_xy = float(xy.sum() / xy.shape[0])
        sqrx = x*x
        k = (avg_xy-avg_y*avg_x)
        if not sqrx.sum()==0 and not avg_x==0:
            k = k / ((sqrx.sum()/sqrx.shape[0]-avg_x**2))
        else:
            k = self.detect_msg.error_k
        return k 
        
        # ostu thresh find
    def ostu(self,image):
        blur = cv2.GaussianBlur(image,(5,5),0)

        # find normalized_histogram, and its cumulative distribution function
        hist = cv2.calcHist([blur],[0],None,[256],[0,256])
        hist_norm = hist.ravel()/hist.sum()
        Q = hist_norm.cumsum()
        bins = np.arange(256)
        fn_min = np.inf
        thresh = -1
        for i in range(1,256):
            p1,p2 = np.hsplit(hist_norm,[i]) # probabilities
            q1,q2 = Q[i],Q[255]-Q[i] # cum sum of classes
            if q1 < 1.e-6 or q2 < 1.e-6:
                continue
            b1,b2 = np.hsplit(bins,[i]) # weights
            # finding means and variances
            m1,m2 = np.sum(p1*b1)/q1, np.sum(p2*b2)/q2
            v1,v2 = np.sum(((b1-m1)**2)*p1)/q1,np.sum(((b2-m2)**2)*p2)/q2
            # calculates the minimization function
            fn = v1*q1 + v2*q2
            if fn < fn_min:
                fn_min = fn
                thresh = i
        return thresh


    def start_callback(self,req):
        self.active = req.active
        res = StartUpResponse()
        if self.active:
            res.result = "Go"
        else:
            res.result = "Stop"
        
        return res

    def search_leftbound(self, image):
        left_list=[]
        (height, width) = image.shape
        for i in range(width):
            if image[height-1][i]==255:
                left_list.append(i)
                break
        if len(left_list)==0:
            return -1
        for h in range(2,height+1):
            end_flag = False
            if image[height-h][left_list[-1]] == 255:
                if left_list[-1]==0:
                    left_list.append(0)
                    continue
                for i in range(1,left_list[-1]+1):
                    if image[height-h][left_list[-1]-i]==0:
                        left_list.append(left_list[-1]-i+1)
                        break
                    if (left_list[-1]-i) == 0:
                        left_list.append(0)
            else :
                for i in range(1,width-left_list[-1]):
                    if image[height-h][left_list[-1]+i]==255:
                        left_list.append(left_list[-1]+i)
                        break
                    if (left_list[-1]+i)==(width-1):
                        end_flag = True
            if end_flag == True:
                break
        return left_list

    def search_rightbound(self,image):
        right_list=[]
        (height, width) = image.shape
        for i in range(width):    
            if image[height-1][width-1-i]==255:
                right_list.append(width-1-i)
                break
        if len(right_list)==0:
            return -1
        for h in range(2,height+1):
            end_flag = False
            if image[height-h][right_list[-1]] == 255:
                if right_list[-1] == width-1:
                    right_list.append(width-1)
                    continue
                for i in range(1,width-right_list[-1]):
                    if image[height-h][right_list[-1]+i]==0:
                        right_list.append(right_list[-1]+i-1)
                        break
                    if (right_list[-1]+i) == (width-1):
                        right_list.append(width-1)
            else :
                for i in range(1,right_list[-1]+1):
                    if image[height-h][right_list[-1]-i]==255:
                        right_list.append(right_list[-1]-i)
                        break
                    if (right_list[-1]-i)==0:
                        end_flag = True
            if end_flag == True:
                break

        return right_list

    def cal_curve_error(self,x1,x2,x3,y1,y2,y3):
        a = np.sqrt((x2-x3)**2+(y2-y3)**2) #distance x2 x3
        b = np.sqrt((x1-x3)**2+(y1-y3)**2) #distance x1 x3
        c = np.sqrt((x1-x2)**2+(y1-y2)**2) #distance x1 x2

        if not a==0 and not b==0 and not c==0:

            cosB = (a**2+c**2-b**2)/(2*a*c)
            if cosB > 1 :
                cosB=1
            if cosB < -1 :
                cosB=-1
            if (x1+x2)/2<x3:
                sinB = np.sqrt(1-cosB*2)
            else :
                sinB = -np.sqrt(1-cosB*2)
            cur = 2*sinB / b
            cur = math.atan(cur)
            real_cur = self.detect_msg.w_z / (self.detect_msg.v_x+1e-3)
            real_cur = math.atan(real_cur)
            # print(cur)
            # print(real_cur)
            err_cur = real_cur - cur
            return err_cur
        else :
            return self.err_cur

def main():
    rospy.init_node("env_feedback_node")
    image_object = EnvFeedback()

    def shutdown():
        print("showdown time")
        cv2.destroyAllWindows()
        
    rospy.on_shutdown(shutdown)

    s = rospy.Service('/Activate', StartUp, image_object.start_callback)
    count = 0
    while not rospy.is_shutdown():

        if not image_object.active:
            continue

        image_object.detect_pub.publish(image_object.detect_msg)

        # if image_object.detect_msg.failed_flag == 1:
        #     image_object.reset_env()

        if count >= 5 :
            # rospy.loginfo("error_x = " + str(image_object.detect_msg.error_x))
            # rospy.loginfo("error_k = "+ str(image_object.detect_msg.error_k))
            # rospy.loginfo("dis =  "+ str(image_object.detect_msg.distance))
            count = 0
        count += 1
        # rospy.loginfo("v_x = " + str(image_object.detect_msg.v_x))
        # rospy.loginfo("w_z = " + str(image_object.detect_msg.w_z))
        image_object.pub_rate.sleep()


if __name__ == '__main__':
    main()


        