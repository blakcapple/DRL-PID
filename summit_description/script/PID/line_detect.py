#! /usr/bin/env python
import rospy
import cv2 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from summit_description.msg import LineInfo 
import numpy as np

class LineDetect:
    
    def __init__(self):

        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("iRobot/camera/image_raw", Image,
                                self.camera_callback)
        
        self.detect_pub = rospy.Publisher("line_detect", LineInfo, queue_size=10)
        self.detect_msg = LineInfo()
        self.pub_rate = rospy.Rate(10)

    def camera_callback(self, data):
        
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e :
            print(e)

        (height, width, channels) = cv_image.shape
        cv2.imshow("cv_image", cv_image)
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        upper_black = np.array([30,30,30])
        lower_black = np.array([0,0,0])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10']/m['m00'], m['m10']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2

        cv2.circle(mask, (int(cx), int(cy)), 10, (0,0,0), -1)
        cv2.imshow("mask", mask)

        cv2.waitKey(1)

        self.detect_msg.error_x = width/2 - cx
        self.detect_pub.publish(self.detect_msg)
        rospy.loginfo("error_x = " + str(self.detect_msg.error_x))
        self.pub_rate.sleep()

def main():
    rospy.init_node("line_detect_node")
    image_object = LineDetect()
    rospy.spin()
    # ctrl_c = False
    # def shutdown():
    #     ctrl_c = True
    def shutdown():
        print("showdown time")
        cv2.destroyAllWindows()
    rospy.on_shutdown(shutdown)
    # while not ctrl_c:
    #     rate.sleep()

if __name__ == '__main__':
    main()


        