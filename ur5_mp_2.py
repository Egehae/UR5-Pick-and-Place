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
import geometry_msgs.msg #import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from moveit_commander.conversions import pose_to_list


from time import sleep
tracker = Tracker()



class ur5_mp:
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points=[]
        self.state_change_time = rospy.Time.now()

        self.N = 0
        self.i = 0
        self.j = 0
        self.detection = False
        self.callback_flag = True


        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        ###self.arm.set_planner_id("RRTstarkConfigDefault")

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.set_planning_time(0.5)
        self.arm.set_max_acceleration_scaling_factor(.7)
        self.arm.set_max_velocity_scaling_factor(.7)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = 0.2
        wpose.position.y = 0.05
        wpose.position.z = 0.5
        self.waypoints.append(deepcopy(wpose))


####

        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        self.box_name_1 = ''
        self.box_name_2 = ''
        self.box_name_3 = ''
        self.box_name_4 = ''
        self.box_name_5 = ''
        self.box_name_6 = ''
        self.box_name_7 = ''
        self.box_name_8 = ''
        self.box_name_9 = ''
        self.box_name_10 = ''
        self.cylinder_name = ''
        self.scene = scene

        rospy.sleep(0.5)

        box_pose_1 = geometry_msgs.msg.PoseStamped()
        box_pose_1.header.frame_id = robot.get_planning_frame()
        box_pose_1.pose.position.x = -0.68
        box_pose_1.pose.position.y = 0.39
        box_pose_1.pose.position.z = 0.02
        box_name_1 = "box_1"
        scene.add_box(box_name_1, box_pose_1, size=(0.40, 0.02, 1.65))

        box_pose_2 = geometry_msgs.msg.PoseStamped()
        box_pose_2.header.frame_id = robot.get_planning_frame()
        box_pose_2.pose.position.x = -0.68
        box_pose_2.pose.position.y = 0.13
        box_pose_2.pose.position.z = 0.02
        box_name_2 = "box_2"
        scene.add_box(box_name_2, box_pose_2, size=(0.40, 0.02, 1.65))

        box_pose_3 = geometry_msgs.msg.PoseStamped()
        box_pose_3.header.frame_id = robot.get_planning_frame()
        box_pose_3.pose.position.x = -0.68
        box_pose_3.pose.position.y = -0.13
        box_pose_3.pose.position.z = 0.02
        box_name_3 = "box_3"
        scene.add_box(box_name_3, box_pose_3, size=(0.40, 0.02, 1.65))

        box_pose_4 = geometry_msgs.msg.PoseStamped()
        box_pose_4.header.frame_id = robot.get_planning_frame()
        box_pose_4.pose.position.x = -0.68
        box_pose_4.pose.position.y = -0.39
        box_pose_4.pose.position.z = 0.02
        box_name_4 = "box_4"
        scene.add_box(box_name_4, box_pose_4, size=(0.40, 0.02, 1.65))
#
        box_pose_5 = geometry_msgs.msg.PoseStamped()
        box_pose_5.header.frame_id = robot.get_planning_frame()
        box_pose_5.pose.position.x = 0.0
        box_pose_5.pose.position.y = 0.0
        box_pose_5.pose.position.z = -0.4
        box_name_5 = "box_5"
        scene.add_box(box_name_5, box_pose_5, size=(0.15, 0.15, 0.8))
#
        box_pose_6 = geometry_msgs.msg.PoseStamped()
        box_pose_6.header.frame_id = robot.get_planning_frame()
        box_pose_6.pose.position.x = -0.68
        box_pose_6.pose.position.y = 0.0
        box_pose_6.pose.position.z = 0.835
        box_name_6 = "box_6"
        scene.add_box(box_name_6, box_pose_6, size=(0.40, 0.80, 0.02))

        box_pose_7 = geometry_msgs.msg.PoseStamped()
        box_pose_7.header.frame_id = robot.get_planning_frame()
        box_pose_7.pose.position.x = -0.68
        box_pose_7.pose.position.y = 0.0
        box_pose_7.pose.position.z = 0.575
        box_name_7 = "box_7"
        scene.add_box(box_name_7, box_pose_7, size=(0.40, 0.80, 0.02))

        box_pose_8 = geometry_msgs.msg.PoseStamped()
        box_pose_8.header.frame_id = robot.get_planning_frame()
        box_pose_8.pose.position.x = -0.68
        box_pose_8.pose.position.y = 0.0
        box_pose_8.pose.position.z = 0.315
        box_name_8 = "box_8"
        scene.add_box(box_name_8, box_pose_8, size=(0.40, 0.80, 0.02))

        box_pose_9 = geometry_msgs.msg.PoseStamped()
        box_pose_9.header.frame_id = robot.get_planning_frame()
        box_pose_9.pose.position.x = -0.68
        box_pose_9.pose.position.y = 0.0
        box_pose_9.pose.position.z = 0.055
        box_name_9 = "box_9"
        scene.add_box(box_name_9, box_pose_9, size=(0.40, 0.80, 0.02))

        box_pose_10 = geometry_msgs.msg.PoseStamped()
        box_pose_10.header.frame_id = robot.get_planning_frame()
        box_pose_10.pose.position.x = -0.68
        box_pose_10.pose.position.y = 0.0
        box_pose_10.pose.position.z = -0.205
        box_name_10 = "box_10"
        scene.add_box(box_name_10, box_pose_10, size=(0.40, 0.80, 0.02))


####

        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = -pi
        self.default_joint_states[1] = -pi*3/4
        self.default_joint_states[2] = pi*3/4
        self.default_joint_states[3] = -pi
        self.default_joint_states[4] = -pi/2
        self.default_joint_states[5] = 0

        self.arm.set_joint_value_target(self.default_joint_states)
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        self.arm.execute(plan)


        self.end_joint_states = deepcopy(self.default_joint_states)
        self.end_joint_states[0] = 0
        self.end_joint_states[1] = -pi/2
        self.end_joint_states[2] = pi/2
        self.end_joint_states[3] = -pi/2
        self.end_joint_states[4] = -pi/2
        self.end_joint_states[5] = 0

        self.drop_pose = deepcopy(start_pose)
        self.drop_pose.position.x = 0.5
        self.drop_pose.position.y = -0.0
        self.drop_pose.position.z = -0.2
        self.drop_pose.orientation.x = 0.0
        self.drop_pose.orientation.y = 0.707
        self.drop_pose.orientation.z = 0.0
        self.drop_pose.orientation.w = 0.707


        self.track_flag = True

####

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def tracking_callback(self, msg):

      if self.callback_flag:
        #self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y

        if self.N < 12:
            self.execute()
        
        if self.N == 12:
            self.arm.set_joint_value_target(self.default_joint_states)
            self.arm.set_start_state_to_current_state()
            plan = self.arm.plan()
            self.arm.execute(plan)
            self.callback_flag = False


    def execute(self):
        if self.track_flag:
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose
            wpose = deepcopy(start_pose)

            if not self.detection:
              self.i = self.N % 3
              self.j = self.N // 3

              wpose.position.x = -0.4
              wpose.position.y = -0.26+(self.i)*0.26
              wpose.position.z = 0.705-(self.j)*0.26-0.1
              wpose.orientation.x = 0.0
              wpose.orientation.y = 0.0
              wpose.orientation.z = 1.0
              wpose.orientation.w = 0.0

              print self.i
              print self.j
              print wpose

              self.arm.set_pose_target(wpose)
              self.arm.set_start_state_to_current_state()
              plan = self.arm.plan()
              self.arm.execute(plan)

              self.detection = True

            else:
              if not tracker.flag2:
                  self.arm.set_max_acceleration_scaling_factor(.5)
                  self.arm.set_max_velocity_scaling_factor(.5)


                  wpose.position.x = -0.48
                  wpose.position.z -= self.error_y*0.04/105

                  self.arm.set_pose_target(wpose)
                  self.arm.set_start_state_to_current_state()
                  plan = self.arm.plan()
                  self.arm.execute(plan)
   

                  tracker.flag2 = 1
                  self.cxy_pub.publish(tracker)
                  rospy.sleep(0.1)

              else:
                  self.waypoints = []
                  wpose.position.x += 0.24
                  self.waypoints.append(deepcopy(wpose))

                  self.arm.set_start_state_to_current_state()
                  plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                  self.arm.execute(plan)


                  self.arm.set_joint_value_target(self.end_joint_states)
                  self.arm.set_start_state_to_current_state()
                  plan = self.arm.plan()
                  self.arm.execute(plan)


                  self.arm.set_max_acceleration_scaling_factor(.7)
                  self.arm.set_max_velocity_scaling_factor(.7)


                  self.waypoints = []
                  self.waypoints.append(deepcopy(self.drop_pose))
                  self.arm.set_start_state_to_current_state()
                  plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.2, 0.0, True)
                  self.arm.execute(plan)


                  tracker.flag2 = 0
                  self.cxy_pub.publish(tracker)
                  self.detection = False
                  self.N += 1
                  rospy.sleep(0.1)


                  self.arm.set_joint_value_target(self.default_joint_states)
                  self.arm.set_start_state_to_current_state()
                  plan = self.arm.plan()
                  self.arm.execute(plan)

####

mp=ur5_mp()

rospy.spin()
