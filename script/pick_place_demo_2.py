#! /usr/bin/env python

import sys

import rospy
import actionlib

import std_msgs.msg
import geometry_msgs.msg
import move_base_msgs.msg
import pan_tilt_msgs.msg

import move_client
import jaco2_pick_place

def main():
    body_pub = rospy.Publisher('caster/body_controller/command', std_msgs.msg.Float64, queue_size=10)
    head_pub = rospy.Publisher('pan_tilt_driver_node/pan_tilt_cmd', pan_tilt_msgs.msg.PanTiltCmd, queue_size=10)
    
    rospy.init_node('pick_place_demo_node')
    rospy.sleep(2)

    rospy.loginfo("set height to 0.1")
    body_pub.publish(0.1)

    rospy.loginfo('set head pitch to 30')
    head_pub.publish(pan_tilt_msgs.msg.PanTiltCmd(pitch=30.0, yaw=0.0, speed=20))
    rospy.sleep(4)

    pick_place = jaco2_pick_place.Jaco2PickPlace('j2n6s200')
    m_client = move_client.MoveClient('move_base', 'dock_test')

    rospy.loginfo('set arm to ready pose')
    if not pick_place.ready():
        return

    rospy.loginfo('move to pick place')
    if not m_client.move('pick'):
        return
    rospy.sleep(2)

    rospy.loginfo('pick')
    if not pick_place.pick():
        return

    rospy.loginfo('move to place place')
    if not m_client.move('place'):
        return
    rospy.sleep(2)

    rospy.loginfo('place')
    if not pick_place.place():
        return
    rospy.sleep(1)

    rospy.loginfo('set arm to home pose')
    if not pick_place.home():
        return

    rospy.loginfo('move to home place')
    if not m_client.move('home'):
        return

if __name__ == '__main__':
    main()
