#!/usr/bin/env python

import rospy, sys
import numpy as np
# import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_grasping.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class Listener:
    def __init__(self, topic):
        rospy.init_node("Listener", anonymous = False)
        
        self.depth_topic = topic
        # self.color_topic = '/camera/color/image_raw'
        # self.color_sub = rospy.Subscriber(color_topic, Image, color_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        self.bridge = cv_bridge.CvBridge()

    def depth_callback(self, depth_msg):
        try:       
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
            # depth_img = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            print("depth_msg.encoding: %s" % (depth_msg.encoding))
            # pix = (320, 240)
            pix = (407, 236)
            print('%s: Depth at center(%d, %d): %f(mm)\r' % ('/camera/depth/image_rect_raw/', pix[0], pix[1], depth_img[pix[1], pix[0]]))
            print(type(depth_img[pix[1], pix[0]]))
            cv2.namedWindow("depth_map", 3)
            cv2.imshow("depth_map", depth_img)
            # show color
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)
            return

def main():
    depth_topic = '/camera/depth/image_rect_raw'
    listener = Listener(depth_topic)
    rospy.spin()

if __name__=='__main__':
    main()