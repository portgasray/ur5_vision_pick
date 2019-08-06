#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, math, moveit_msgs.msg
import numpy as np
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from ur5_grasping.msg import Tracker
from moveit_python import PlanningSceneInterface
# from sensor_msgs.msg import Image
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep

def tracking_callback(msg):
    track_flag = msg.flag1
    cx = msg.x
    cy = msg.y
    error_x = msg.error_x
    error_y = msg.error_y   

if __name__ == "__main__":

    rospy.init_node("ur5_mp", anonymous=False)
    cxy_sub = rospy.Subscriber('camera_xy', Tracker, tracking_callback, queue_size=1)

    # object_cnt = 0
    # track_flag = False
    # default_pose_flag = True
    # cx = 400.0
    # cy = 400.0
    state_change_time = rospy.Time.now()
    rospy.loginfo("Starting node moveit_cartesian_path")
    # rospy.on_shutdown(cleanup)

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)
    # Initialize the move group for the ur5_arm
    arm = moveit_commander.MoveGroupCommander('arm')
    # Get the name of the end-effector link
    end_effector_link = arm.get_end_effector_link()
    # Set the reference frame for pose targets
    reference_frame = "/base_link"
    # Set the ur5_arm reference frame accordingly
    arm.set_pose_reference_frame(reference_frame)
    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10)

    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.1)
    arm.set_planning_time(0.1)
    arm.set_max_acceleration_scaling_factor(.5)
    arm.set_max_velocity_scaling_factor(.5)

    # Set arm back to home pose
    # arm.set_named_target('home')
    # arm.go()
    # rospy.sleep(1)
    # Specify default (idle) joint states
    default_joint_states = arm.get_current_joint_values()
    default_joint_states[0] = 1.5564
    default_joint_states[1] = -1.5700
    default_joint_states[2] = -1.4161
    default_joint_states[3] = -1.7574
    default_joint_states[4] = 1.6918
    default_joint_states[5] = 0.0
    arm.set_joint_value_target(default_joint_states)
    # Set the internal state to the current state
    arm.set_start_state_to_current_state()
    plan = arm.plan()
    arm.execute(plan)
    rospy.sleep(1)

    # while not rospy.is_shutdown():
    #     # print("not shutdown")
    #     rate.sleep()
    #     try:
    #         t = tf_listener.getLatestCommonTime('/base_link', "ur5_camera_to_robot")
    #         (trans,rot) = listener.lookupTransform('/base_link', "ur5_camera_to_robot", t)
    #         print "ur5_camera_to_robot:" + str(trans)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue
    # print("world co-ordinates in the camera frame x,y : (%s,%s)" %(error_x, error_y))

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.0
    pose_goal.position.y = 500
    pose_goal.position.z = 62

    arm.set_pose_target(pose_goal)
    