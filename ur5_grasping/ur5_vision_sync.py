#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_grasping.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
import message_filters
# from sensor_msgs.msg import Image as msg_Image

import pyrealsense2 as rs

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
tracker = Tracker()

def nothing(x):
    pass

# cv2.namedWindow("Trackbars")

# cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

# def initialize_camera():
#     #start the frames pipe
#     p = rs.pipeline()
#     conf = rs.config()
#     CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640,480,30
#     conf.enable_stream(rs.stream.depth, CAM_WIDTH, CAM_HEIGHT, rs.format.z16, CAM_FPS)
#     conf.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.rgb8, CAM_FPS)
#     # conf.enable_stream(rs.stream.infrared,1, CAM_WIDTH, CAM_HEIGHT, rs.format.y8, CAM_FPS)
#     # conf.enable_stream(rs.stream.accel,rs.format.motion_xyz32f,250)
#     # conf.enable_stream(rs.stream.gyro,rs.format.motion_xyz32f,200)
#     prof = p.start(conf)
#     return p

class ur5_vision:
    def __init__(self, depth_topic, color_topic):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True
        self.cX = 400.0
        self.cY = 400.0
        self.bridge = cv_bridge.CvBridge()
        self.depth_topic = depth_topic
        self.color_topic = color_topic
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.color_sub = message_filters.Subscriber(self.color_topic, Image) 
        self.sync = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.color_sub], queue_size = 5, slop = 0.1)
        self.sync.registerCallback(self.image_callback)

        self.cxy_pub = rospy.Publisher('camera_xy', Tracker, queue_size=1)
        # self.topic = topic
        # self.bridge = CvBridge()

    # def imageDepthCallback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    #         pix = (self.cX, self.cY)
    #         print('%s: Depth at center(%d, %d): %f(mm)\r' % (self.depth_topic, pix[0], pix[1], cv_image[pix[1], pix[0]]))
    #         # sys.stdout.flush()
    #     except CvBridgeError as e:
    #         print(e)
    #         return

    def image_callback(self, depth_msg, color_msg):

        # BEGIN BRIDGE
        try:
            image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
        except CvBridgeError as e:
            print(e)
            return 
        
        # END BRIDGE
        h, w, d = image.shape
        # img_center_x = w/2
        # img_center_y = h/2
        img_center_x = 314.05
        img_center_y = 248.70

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # yellow color
        lower_yellow = np.array([0, 0, 0])
        upper_yellow = np.array([255,255,200])

        # lower_yellow = np.array([l_h, l_s, l_v])
        # upper_yellow = np.array([u_h, u_s, u_v])
        
        wooden_block =  cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        #show result
        # result = cv2.bitwise_and(image, image, mask = wooden_block)

        _, contours, hierarchy = cv2.findContours(wooden_block.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [ c for c in contours if cv2.contourArea(c) > 1000 and cv2.contourArea(c) < h * w * 0.9 ]

        cX = 0
        cY = 0
        for cnt in cnts:
            # bounding box
            # x, y, w, h = cv2.boundingRect(cnt)
            # cv2.rectangle(image,(x,y), (x+w,y+h), (0,0,255), 3)

            ## center of contour
            M = cv2.moments(cnt)
            # M = cv2.moments(thresh)
            # calculate x, y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            
            # drawing contour
            cv2.drawContours(image, [cnt], 0, (0,255,0), 2)
            cv2.circle(image, (cX, cY), 3, (0,0,255), -1)
            cv2.putText(image, "({}, {})".format(int(cX), int(cY)), (int(cX-5), int(cY+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # get the depth
            pix = (cX, cY)
            print('%s: Depth at center(%d, %d): %f(mm)\r' % (self.depth_topic, pix[0], pix[1], depth[pix[1], pix[0]]))

        '''
        calculate the x,y in mm 
        (should be deprojection here)
        '''
        
        # using fixed pixel_x pixel_y 
        pixels_permm_x = 1.3275
        pixels_permm_y = 1.3415
        
        if cX != 0 and cY != 0:
            pixel_x_in_meter =  (cX - img_center_x) / pixels_permm_x
            pixel_y_in_meter = (cY - img_center_y) / pixels_permm_y

        tracker.x = pixel_x_in_meter
        tracker.y = pixel_y_in_meter
        # tracker.z = pixel_z_in_meter

        print("world co-ordinates in the camera frame x, y z mm: (%s,%s)" %(tracker.x, tracker.y))
        self.cxy_pub.publish(tracker)
        
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image)
        cv2.imshow("wooden mask", wooden_block)
        # cv2.imshow("result", result)
        cv2.waitKey(1)

if __name__ == "__main__":
    depth_topic = '/camera/depth/image_rect_raw'
    color_topic = '/camera/color/image_raw'
    listener = ur5_vision(depth_topic, color_topic)
    rospy.spin()
