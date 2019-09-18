#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, math, moveit_msgs.msg
import tf2_ros, tf2_geometry_msgs
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import *
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from ur5_grasping.msg import Tracker
from moveit_python import PlanningSceneInterface
# from sensor_msgs.msg import Image
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep

from gripper import GripperClient


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal=True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True


class UR5_Pick_Up(object):
    def __init__(self):
        super(UR5_Pick_Up, self).__init__()
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_mp', anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator" #arm #manipulator
        # Initialize the move group for the ur5_arm
        group = moveit_commander.MoveGroupCommander(group_name)
        # Set the reference frame for pose targets
        reference_frame = "base_link"
        # Set the ur5_arm reference frame accordingly
        group.set_pose_reference_frame(reference_frame)
        # Allow replanning to increase the odds of a solution
        group.allow_replanning(True)
        
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        planning_frame = group.get_planning_frame()
        print ("============ Reference frame: %s ============ " % planning_frame)
        # Get the name of the end-effector link
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s ============ " % eef_link)
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()
        print "============ Printing robot state"
        print robot.get_current_state()

        # Allow some leeway in position (meters) and orientation (radians)
        group.set_goal_position_tolerance(0.01)
        group.set_goal_orientation_tolerance(0.1)
        group.set_planning_time(0.1)
        group.set_max_acceleration_scaling_factor(.15)
        group.set_max_velocity_scaling_factor(.15)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.track_flag = False
        self.default_pose_flag = True
        
        self.cx = 0
        self.cy = 0
        self.cz = 0

        self.target_point = PointStamped()
        # self.bridge=cv_bridge.CvBridge()
        # self.image_sub=rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        # self.cxy_sub = rospy.Subscriber('camera_xy', Tracker, self.tracking_callback, queue_size=1)
        # self.cxyz_sub = rospy.Subscriber('camera_xyz',Tracker, self.tracking_callback, queue_size=1)
        
        self.cxyz_sub = rospy.Subscriber('position_in_camera', Tracker, self.tracking_callback, queue_size=1)
        self.tf_listener_ = tf.TransformListener()
        rate = rospy.Rate(10)


    def go_to_ready_pose(self):
        # Set arm back to home pose
        # group.set_named_target('home')
        # group.go()
        # rospy.sleep(1)
        # Specify default (idle) joint states
        group = self.group
        default_joint_states = group.get_current_joint_values()
        print(type(default_joint_states), default_joint_states)
        # default_joint_states[0] = 1.5564
        # default_joint_states[1] = -1.5700
        # default_joint_states[2] = -1.4161
        # default_joint_states[3] = -1.7574
        # default_joint_states[4] = 1.6918
        # default_joint_states[5] = 0.0

        default_joint_states[0] = 1.6497
        default_joint_states[1] = -1.5534
        default_joint_states[2] = -1.4492
        default_joint_states[3] = -1.7472
        default_joint_states[4] = 1.6474
        default_joint_states[5] = 0.7089

        # Set the internal state to the current state
        group.go(default_joint_states, wait=True)
        group.stop()
        rospy.sleep(1)
        current_joints = group.get_current_joint_values()
        current_pose = self.group.get_current_pose().pose
        print("current pose: ", current_pose)
        return all_close(default_joint_states, current_joints, 0.01)


    def tracking_callback(self, msg):
        # msg here is 'tracker' from *vision.py
        track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.cz = msg.z
        # print ("receive coordinates from topic: (%s,%s,%s)" %(self.cx, self.cy, self.cz))
    
    def coordinate_convert(self):
        print "Start trans ... ... "
        listener = self.tf_listener_
        if listener.frameExists("base_link") and listener.frameExists("camera_depth_optical_frame"):  #camera_depth_optical_frame
            # t = self.tf_listener_.getLatestCommonTime("base_link", "camera_link")
            # p1 = geometry_msgs.msg.PoseStamped()
            # p1.header.frame_id = "camera_link"
            # p1.pose.orientation.w = 0.5    # Neutral orientation
            # p_in_base = self.tf_listener_.transformPose("base_link", p1)

            # listener.waitForTransform("base_link", "camera_link", rospy.Time(0),rospy.Duration(4.0))
            listener.waitForTransform("base_link", "camera_depth_optical_frame", rospy.Time(0),rospy.Duration(4.0))  
            camera_point = PointStamped()
            camera_point.header.frame_id = "camera_depth_optical_frame"
            camera_point.header.stamp = rospy.Time(0)
            camera_point.point.x = self.cx
            camera_point.point.y = self.cy
            camera_point.point.z = self.cz / 1000
            print "coordinate in camera frame (%s, %s, %s)" %(camera_point.point.x, camera_point.point.y, camera_point.point.z)
            self.target_point = listener.transformPoint("base_link", camera_point)
            print "position in the robot base:"
            print self.target_point


    def go_to_pose_goal(self):
            group = self.group
            current_pose = group.get_current_pose().pose
            print("Current pose: ", current_pose)
            pose_goal=geometry_msgs.msg.Pose()

            pose_goal.orientation.x = 0.5
            pose_goal.orientation.y = 0.5
            pose_goal.orientation.z = -0.5
            pose_goal.orientation.w = 0.5
            
            # print("target point: " ,self.target_point)
            pose_goal.position.x =  self.target_point.point.x #0
            pose_goal.position.y =  self.target_point.point.y  #-0.5
            pose_goal.position.z =  self.target_point.point.z
            ##compensate accuracy
            pose_goal.position.x = pose_goal.position.x + 0.0369
            pose_goal.position.y = pose_goal.position.y + 0.0122
            # pose_goal.position.z =  0.058
            group.set_pose_target(pose_goal)
            plan = group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            current_pose=group.get_current_pose().pose
            print("New current pose: ", current_pose)
            return all_close(pose_goal, current_pose, 0.01)

    def place_block(self):
            group = self.group
            pose_goal=geometry_msgs.msg.Pose()

            pose_goal.orientation.x = 0.5
            pose_goal.orientation.y = 0.5
            pose_goal.orientation.z = -0.5
            pose_goal.orientation.w = 0.5
        
            pose_goal.position.x =  -0.35
            pose_goal.position.y =  -0.68
            pose_goal.position.z =  0.055
            group.set_pose_target(pose_goal)
            plan = group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            current_pose=group.get_current_pose().pose
            print("New current pose: ", current_pose)
            return all_close(pose_goal, current_pose, 0.01)
         

if __name__ == "__main__":
    ur5_pick_up = UR5_Pick_Up()
    gripper = GripperClient()
    # image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    # try:
    print "============ Press `Enter` to go to ready pose"
    raw_input()
    ur5_pick_up.go_to_ready_pose()
    
    ur5_pick_up.coordinate_convert()
    print "============ Press `Enter` to go to pose goal and get the block"
    raw_input()
    ur5_pick_up.go_to_pose_goal()
    ##pick up
    # gripper.open()

    print "============ Press `Enter` to place the block"
    raw_input()
    ur5_pick_up.go_to_ready_pose()
    #place
    ur5_pick_up.place_block()
    # gripper.close()

    print "============ Press `Enter` to go ready pose"
    raw_input()
    ur5_pick_up.go_to_ready_pose()
    
    print "============ Finished"
    # except rospy.ROSInterruptException:
    #     return
    # except KeyboardInterrupt:
    #     return

    
    # position_xy_pub = rospy.Publisher('camera_xy', Tracker, queue_size=1)