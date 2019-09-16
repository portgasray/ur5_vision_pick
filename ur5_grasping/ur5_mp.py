#!/usr/bin/env python

import roslib
roslib.load_manifest('learning_tf')
import rospy, sys, numpy as np
import moveit_commander
import geometry_msgs.msg
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_grasping.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep
import math
import tf
tracker = Tracker()



class ur5_mp:
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        self.cxy_sub = rospy.Subscriber('camera_xy', Tracker, self.tracking_callback, queue_size=1)
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0

        self.points=[]
        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)
        
        tf_listener = tf.TransformListener()

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('arm')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.5)

        # Set arm back to home pose
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)

        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = 1.5564
        self.default_joint_states[1] = -1.5700
        self.default_joint_states[2] = -1.4161
        self.default_joint_states[3] = -1.7574
        self.default_joint_states[4] = 1.6918
        self.default_joint_states[5] = 0.0

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        self.arm.execute(plan)
        rospy.sleep(1)


    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def tracking_callback(self, msg):

        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        
        print("X-Co-ordinate in Camera Frame :", self.error_x)
        print("Y-Co-ordinate in Camera Frame :", self.error_y)
        
        # if len(self.pointx)>9:
        #     self.track_flag = True
        # if self.phase == 2:
        #     self.track_flag = False
        #     self.phase = 1

        # if (self.track_flag and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
        #     self.execute()
        #     self.default_pose_flag = False
        # else:
        #     if not self.default_pose_flag:
        #         self.track_flag = False
        #         self.execute()
        #         self.default_pose_flag = True



    def execute(self):
        if self.track_flag:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            # Move target state
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.w = 1.0
            pose_goal.position.x = 0.4
            pose_goal.position.y = 0.1
            pose_goal.position.z = 0.4
            self.arm.set_pose_target(pose_goal)
            plan = self.arm.go(wait=True)
            self.arm.execute(plan)
            self.arm.stop()
            self.arm.clear_pose_targets()

mp=ur5_mp()

rospy.spin()
