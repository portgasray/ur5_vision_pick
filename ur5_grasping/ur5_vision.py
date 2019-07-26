#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

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
        cnts = [ cnts[i] for i in child_contour if (cv2.contourArea(cnts[i]) > 100) and (cv2.contourArea(cnts[i]) < 1000)]
        cX = 0 
        cY = 0
        for c in cnts:
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
        
        self.track_flag = True
        self.cx = cX
        self.cy = cY

        tracker.x = cX
        tracker.y = cY

        self.cxy_pub.publish(tracker)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

if __name__ == "__main__":
    follower=ur5_vision()
    rospy.spin()
