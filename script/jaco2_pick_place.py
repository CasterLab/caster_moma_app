#! /usr/bin/env python

import sys
import math
import numpy as np

import tf
import rospy
import actionlib

import std_msgs.msg
import kinova_msgs.msg
import geometry_msgs.msg

ready_angles = {}
ready_angles['j2s6s200'] = [92.9301071167, 272.362121582, 49.6396369934, 94.7798538208, 272.449554443, 46.2744293213, 0.0]

home_angles = {}
home_angles['j2s6s200'] = [91.8869247437, 274.86239624, 105.129676819, 1.35414087772, 281.996032715, 92.4805374146, 0.0]


class Jaco2PickPlace():
    def __init__(self, arm_name):
        self.__arm_name = arm_name
        self.__pose_action_client = actionlib.SimpleActionClient(self.__arm_name+'_driver/pose_action/tool_pose',
                                                               kinova_msgs.msg.ArmPoseAction)
        self.__joints_action_client = actionlib.SimpleActionClient(self.__arm_name+'_driver/joints_action/joint_angles',
                                                                 kinova_msgs.msg.ArmJointAnglesAction)
        self.__gripper_action_client = actionlib.SimpleActionClient(self.__arm_name+'_driver/fingers_action/finger_positions',
                                                                  kinova_msgs.msg.SetFingersPositionAction)

        self.__pose_action_client.wait_for_server()
        self.__joints_action_client.wait_for_server()
        self.__gripper_action_client.wait_for_server()

    def __set_joint_angle(self, target_angle):
        goal = kinova_msgs.msg.ArmJointAnglesGoal()
        goal.angles.joint1 = target_angle[0]
        goal.angles.joint2 = target_angle[1]
        goal.angles.joint3 = target_angle[2]
        goal.angles.joint4 = target_angle[3]
        goal.angles.joint5 = target_angle[4]
        goal.angles.joint6 = target_angle[5]
        goal.angles.joint7 = target_angle[6]

        self.__joints_action_client.send_goal(goal)
        if self.__joints_action_client.wait_for_result(rospy.Duration(20.0)):
            return self.__joints_action_client.get_result()
        else:
            print('the joint angle action timed-out')
            self.__joints_action_client.cancel_all_goals()
            return None

    def __set_pose(self, target_pose):
        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose = target_pose

        self.__pose_action_client.send_goal(goal)
        if self.__pose_action_client.wait_for_result(rospy.Duration(10.0)):
            return self.__pose_action_client.get_result()
        else:
            self.__pose_action_client.cancel_all_goals()
            print('the cartesian action timed-out')
            return None

    def __set_gripper(self, target_percent):
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = 6800.0 * target_percent
        goal.fingers.finger2 = 6800.0 * target_percent
        goal.fingers.finger3 = 0

        self.__gripper_action_client.send_goal(goal)
        if self.__gripper_action_client.wait_for_result(rospy.Duration(5.0)):
            return self.__gripper_action_client.get_result()
        else:
            self.__gripper_action_client.cancel_all_goals()
            rospy.logwarn('the gripper action timed-out')
            return None

    def home(self):
        try:
            result = self.__set_joint_angle(home_angles[self.__arm_name])
        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True

    def ready(self):
        try:
            result = self.__set_joint_angle(ready_angles[self.__arm_name])
        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True

    def pick(self):
        try:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header = std_msgs.msg.Header(frame_id='object')

            t = tf.transformations.quaternion_from_euler(math.pi/2.0, 0.0, math.pi/2.0)
            pose.pose.orientation = geometry_msgs.msg.Quaternion(x=t[0], y=t[1], z=t[2], w=t[3])

            result = self.__set_joint_angle(ready_angles[self.__arm_name])
            rospy.Rate(0.4).sleep()

            result = self.__set_gripper(0.1)
            # rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.15, y=0.0, z=0.03)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.05, y=0.0, z=0.03)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            result = self.__set_gripper(0.8)
            # rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.20, y=0.0, z=0.10)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            result = self.__set_joint_angle(ready_angles[self.__arm_name])
            rospy.Rate(1).sleep()

            # result = self.__set_gripper(0.1)

        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True

    def place(self):
        try:
            pose = geometry_msgs.msg.PoseStamped()
            pose.header = std_msgs.msg.Header(frame_id='plane')

            t = tf.transformations.quaternion_from_euler(0.0, math.pi/2.0, 0.0)
            pose.pose.orientation = geometry_msgs.msg.Quaternion(x=t[0], y=t[1], z=t[2], w=t[3])

            result = self.__set_joint_angle(ready_angles[self.__arm_name])
            rospy.Rate(0.4).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.05, y=0.1, z=0.0)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.05, y=0.055, z=0.0)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            result = self.__set_gripper(0.1)
            rospy.Rate(1).sleep()

            pose.pose.position = geometry_msgs.msg.Point(x=-0.20, y=0.15, z=0.0)
            result = self.__set_pose(pose)
            # rospy.Rate(1).sleep()

            result = self.__set_joint_angle(ready_angles[self.__arm_name])
            rospy.Rate(1).sleep()

            # result = self.__set_gripper(0.1)

        except rospy.ROSInterruptException:
            print "program interrupted before completion"
            return False

        return True
