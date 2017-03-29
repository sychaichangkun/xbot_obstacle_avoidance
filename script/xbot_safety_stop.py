#!/usr/bin/env python
# coding=utf-8
"""
xbot safety stop
"""

import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class xbot_safety_stop():
    """
    when obstacle detected, just stop
    """

    def __init__(self):
        rospy.init_node('xbot_safety_stop')
        self.define()
        self.pub = rospy.Publisher(
            '/cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_dataCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~SafeLAng'):
            rospy.set_param('~SafeLAng', 130)
        self.SafeLAng = rospy.get_param('~SafeLAng')

        if not rospy.has_param('~SafeRAng'):
            rospy.set_param('~SafeRAng', 230)
        self.SafeRAng = rospy.get_param('~SafeRAng')

        if not rospy.has_param('~SafeDist'):
            rospy.set_param('~SafeDist', 0.5)
        self.SafeDist = rospy.get_param('~SafeDist')

        print "==========Settings======== "
        print "SafeAng:  [", self.SafeLAng, ',', self.SafeRAng, ']'
        print "SafeDist:  ",self.SafeDist

    def clear(self):
        rospy.delete_param('~SafeLAng')
        rospy.delete_param('~SafeRAng')
        rospy.delete_param('~SafeDist')    
        rospy.delete_param('~RetreatTime')
        rospy.delete_param('~WaitToRetreatTime')     

    def is_danger(self, scan_data):
        if min(scan_data.ranges[self.SafeLAng:self.SafeRAng]) < 0.5:
            return True
        return False


    def scan_dataCB(self, scan_data):
        if self.is_danger(scan_data):
            print "obstacle detected"
            self.pub.publish(Twist())


if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        x = xbot_safety_stop()
        x.clear()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("robot twist node terminated.")
