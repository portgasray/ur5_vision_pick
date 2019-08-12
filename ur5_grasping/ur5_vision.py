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

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # black color
        black = 97
        sensitivity = 15
        lower_black = np.array([black - sensitivity, 189, 0])
        upper_black = np.array([black + sensitivity,255,255])
        a4_paper =  cv2.inRange(hsv, lower_black, upper_black)

        _, contours, hierarchy = cv2.findContours(a4_paper.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [ c for c in contours if cv2.contourArea(c) > 1000 and cv2.contourArea(c) < h * w * 0.9 ]

        paper_index = 0
        block_index = 1

        x, y, w, h = cv2.boundingRect(cnts[paper_index])
        cv2.rectangle(image,(x,y), (x+w,y+h), (0,0,255), 3)

        pixels_permm_x = w/290
        pixels_permm_y = h/202

        ## center of block
        block_cnt = cnts[block_index]
        M = cv2.moments(block_cnt)
        # M = cv2.moments(thresh)
        # calculate x, y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

        # drawing contour
        cv2.drawContours(result, [block_cnt], 0, (0,255,0), 2)
        cv2.circle(image, (cX, cY), 3, (0,0,255), -1)
        cv2.putText(image, "({}, {})".format(int(cX), int(cY)), (int(cX-5), int(cY+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # cX = 0
        # cY = 0

        cX = (cX - img_center_x) / pixels_permm_x
        cY = (cY - img_center_y) / pixels_permm_y

        tracker.x = cX
        tracker.y = cY


        print("world co-ordinates in the camera frame x: (%s,%s)" %(tracker.x, tracker.y))
        self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

if __name__ == "__main__":
    follower=ur5_vision()
    rospy.spin()
