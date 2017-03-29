#!/usr/bin/env python
# coding=utf-8
"""
xbot_FSM.py
"""


import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from xbot_msgs.msg import DockInfraRed
from sensor_msgs.msg import LaserScan
from collections import deque


class xbot_FSM():
    """
    xbot Finite-State Machine
    States:'Advancing', 'Stopped', 'Retreating', 'Waiting' 
    When specific event is triggered, xbot state will transform 
    from current state to another, with some actions performed
    """

    def __init__(self):
        rospy.init_node('xbot_FSM')
        self.state = ()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbot_FSM()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("xbot_FSM terminated.")
