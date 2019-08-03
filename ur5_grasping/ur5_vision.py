#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_grasping.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
tracker = Tracker()
class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.cxy_pub = rospy.Publisher('camera_xy', Tracker, queue_size=1)


    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        h, w, d = image.shape
        img_center_x = w/2
        img_center_y = h/2
        # convert image to grayscale image
        gray_image =  cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        kernel_size = 13
        radius = 0
        blurred_image = cv2.GaussianBlur(gray_image , (kernel_size, kernel_size) , radius)

        #convert the grayscale image to binary image
        threshold = 136
        max_value = 255
        ret_, thresh = cv2.threshold(blurred_image, threshold, max_value, cv2.THRESH_BINARY)

        #Finding contours
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        child_contour = hierarchy [0, :,2]
        cnts = contours
        paper_cnt = contours

        cnts = [ cnts[i] for i in child_contour ]
        cX = 0 
        cY = 0
        for i, c in enumerate(cnts):
            area = cv2.contourArea(c) 
            if area > 100:
                self.track_flag = True
                # calculate momnets of binary image
                M = cv2.moments(c)
                # calculate x, y coordinate of center
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(image,(x,y), (x+w,y+h), (0,0,255), 3)
                #draw a circle center
                cv2.circle(image, (cX, cY), 1, (255, 255, 255), -1)
                cv2.putText(image, "({}, {})".format(int(cX), int(cY)), (int(cX-5), int(cY+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                self.cx = cX
                self.cy = cY

                tracker.x = cX
                tracker.y = cY
                tracker.flag1 = self.track_flag
               
            else:
                self.track_flag = False
                tracker.flag1 = self.track_flag

        for cnt in paper_cnt:
            if cv2.contourArea(cnt) > 5000:
                paper_cnt  = cnt
        bbox_x, bbox_y, bbox_w, bbox_h = cv2.boundingRect(paper_cnt)
        cv2.rectangle(image,(bbox_x,bbox_y),(bbox_x + bbox_w, bbox_y + bbox_h),(255,0,0),1)

        # print("height : ", bbox_h)
        # print("WIDTH  : ", bbox_w)
        pixels_permm_y = bbox_h / 210
        pixels_permm_x = bbox_w  / 297
        
        # cX = (cX - img_center_x) / pixels_permm_x
        # cY = (cY - img_center_y) / pixels_permm_y


        self.error_x = ( self.cx - img_center_x ) / pixels_permm_x
        self.error_y = ( self.cy - img_center_y ) / pixels_permm_y
        tracker.error_x = self.error_x
        tracker.error_y = self.error_y
        
        print("world co-ordinates in the camera frame x: (%s,%s)" %(tracker.error_x, tracker.error_y))
        self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

if __name__ == "__main__":
    follower=ur5_vision()
    rospy.spin()
