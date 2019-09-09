import pyrealsense2 as rs
import numpy as np
import cv2
import geometry_msgs.msg
from ur5_grasping.msg import Tracker
import rospy

def nothing(x):
    pass

def create_clr_trackbar(nothing):
    cv2.namedWindow("Trackbars")

    cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
    cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
    cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
    return

def get_clr_trackbar():
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")

    lower_color = np.array([l_h, l_s, l_v])
    upper_color = np.array([u_h, u_s, u_v])
    return lower_color, upper_color

def initialize_camera():
    #start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    CAM_WIDTH, CAM_HEIGHT, CAM_FPS = 640,480,30
    conf.enable_stream(rs.stream.depth, CAM_WIDTH, CAM_HEIGHT, rs.format.z16, CAM_FPS)
    conf.enable_stream(rs.stream.color, CAM_WIDTH, CAM_HEIGHT, rs.format.rgb8, CAM_FPS)
    prof = p.start(conf)
    # # Getting the depth sensor's depth scale (see rs-align example for explanation)
    # depth_sensor = prof.get_device().first_depth_sensor()
    # depth_scale = depth_sensor.get_depth_scale()
    # print("Depth Scale is: " , depth_scale)
    # # Create an align object
    # align_to = rs.stream.color
    # align = rs.align(align_to)
    # return p, align
    return p

def convert_depth_pixel_to_metric_coordinate(depth, pixel_x, pixel_y, camera_intrinsics):
	"""
	Convert the depth and image point information to metric coordinates
	Parameters:
	-----------
	depth 	 	 	 : double
						   The depth value of the image point
	pixel_x 	  	 	 : double
						   The x value of the image coordinate
	pixel_y 	  	 	 : double
							The y value of the image coordinate
	camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
	Return:
	----------
	X : double
		The x value in meters
	Y : double
		The y value in meters
	Z : double
		The z value in meters
	"""
	X = (pixel_x - camera_intrinsics.ppx)/camera_intrinsics.fx *depth
	Y = (pixel_y - camera_intrinsics.ppy)/camera_intrinsics.fy *depth
	return X, Y, depth

def find_block_contour(image, lower_color, upper_color):
    h, w, d = image.shape
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #wooden block color fixed
    # lower_yellow = np.array([0, 0, 0])
    # upper_yellow = np.array([255,255,129])
    # wooden_block =  cv2.inRange(image, lower_yellow, upper_yellow)
    wooden_block =  cv2.inRange(image, lower_color, upper_color)
    _, contours, hierarchy = cv2.findContours(wooden_block.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [ c for c in contours if cv2.contourArea(c) > 1000 and cv2.contourArea(c) < h * w * 0.9 ]
    return cnts

def find_block_center(contour):
    M = cv2.moments(cnt)
    # calculate x, y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return cX, cY

if __name__ == "__main__":

    tracker = Tracker()
    
    ### create color trackbar for extracting hsv value
    create_clr_trackbar(nothing)

    rospy.init_node("detection", anonymous=False)
    position_block = rospy.Publisher('position_in_camera',Tracker, queue_size=1)
    try:
        pipeline = initialize_camera()
        align_to = rs.stream.color
        align = rs.align(align_to)

        while True:
            
            frames = pipeline.wait_for_frames()
            ### not aligned
            # color_frame = frames.get_color_frame()
            # depth_frame = frames.get_depth_frame()
    
            # if not depth_frame or not color_frame: continue
            # color_image = np.asanyarray(color_frame.get_data())

            ### Align the depth frame to color frame
            aligned_frames = align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame: continue
            
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics

            image = color_image
            # lower_color, upper_color = get_clr_trackbar()
            #wooden block color fixed
            lower_color = np.array([0, 0, 0])
            upper_color = np.array([255,255,129])
            result =  cv2.inRange(image, lower_color, upper_color)

            contours = find_block_contour(image.copy(), lower_color, upper_color)
            for cnt in contours:
                cv2.drawContours(image, [cnt], 0, (0,255,0), 2)
                cX, cY = find_block_center(cnt)
                if cX != 0 and cY != 0:
                    cv2.circle(image, (cX, cY), 3, (0,0,255), -1)
                    cv2.putText(image, "({}, {})".format(int(cX), int(cY)), (int(cX-5), int(cY+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    # get depth from pixel            
                    # depth = depth_frame.get_distance(cX, cY)
                    depth = aligned_depth_frame.get_distance(cX, cY)
                    cv2.putText(image, "{}".format(round(depth, 4)), (cX - 5, cY - 15 ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    # print("pixel of x, y: (%s,%s)" %(cX, cY))

                    depth_point_in_meters_camera_coords = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth)
                    print("camera coordinate of x, y, z: (%.3f, %.3f, %.3f)" %(depth_point_in_meters_camera_coords[0], 
                        depth_point_in_meters_camera_coords[1], depth_point_in_meters_camera_coords[2]))

                    tracker.x = depth_point_in_meters_camera_coords[0]
                    tracker.y = depth_point_in_meters_camera_coords[1]
                    tracker.z = depth_point_in_meters_camera_coords[2]
                    position_block.publish(tracker)

            cv2.namedWindow("window", 1)
            cv2.imshow("window", image)
            cv2.imshow("wooden mask", result)
            cv2.waitKey(1)
            rospy.spin()
    except Exception as e:
        print(e)
        pass

    finally:
        # Stop streaming
        pipeline.stop()
