#!/usr/bin/env python
import sys, rospy, tf, moveit_commander, math, moveit_msgs.msg
import numpy as np
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

def tracking_callback(msg):
    track_flag = msg.flag1
    cx = msg.x
    cy = msg.y
    error_x = msg.error_x
    error_y = msg.error_y   

def go_to_ready_pose(group):
    # Set arm back to home pose
    # group.set_named_target('home')
    # group.go()
    # rospy.sleep(1)
    # Specify default (idle) joint states
    default_joint_states = group.get_current_joint_values()
    print(type(default_joint_states), default_joint_states)
    default_joint_states[0] = 1.5564
    default_joint_states[1] = -1.5700
    default_joint_states[2] = -1.4161
    default_joint_states[3] = -1.7574
    default_joint_states[4] = 1.6918
    default_joint_states[5] = 0.0

    # Set the internal state to the current state
    group.go(default_joint_states, wait=True)
    group.stop()
    rospy.sleep(1)
    current_joints = group.get_current_joint_values()
    
    current_pose = group.get_current_pose().pose
    print("current pose: ", current_pose)

    return all_close(default_joint_states, current_joints, 0.01)


def go_to_pose_goal(group):
    current_pose=group.get_current_pose().pose
    print("Current pose: ", current_pose)
    pose_goal=geometry_msgs.msg.Pose()

    # pose_goal.orientation.x= 0.5
    # pose_goal.orientation.y= 0.5
    # pose_goal.orientation.z= -0.5
    # pose_goal.orientation.w= 0.5

    pose_goal.orientation.x= 0.5
    pose_goal.orientation.y= 0.5
    pose_goal.orientation.z= -0.5
    pose_goal.orientation.w= 0.5
    
    pose_goal.position.x= 0.0 #0
    pose_goal.position.y= -0.5 #-0.5
    pose_goal.position.z= 0.062 #0.44     

    group.set_pose_target(pose_goal)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    current_pose=group.get_current_pose().pose
    print("New current pose: ", current_pose)
    return all_close(pose_goal, current_pose, 0.01)

def main(arm):
    try:
        print "============ Press `Enter` to go to ready pose"
        raw_input()
        go_to_ready_pose(arm)

        print "============ Press `Enter` to go to pose goal and get the box"
        raw_input()
        go_to_pose_goal(arm)
        print "============ Finished"

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

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
    # arm = moveit_commander.MoveGroupCommander('arm')
    arm = moveit_commander.MoveGroupCommander('manipulator')
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
    arm.set_max_acceleration_scaling_factor(.15)
    arm.set_max_velocity_scaling_factor(.15)
    
    main(arm)