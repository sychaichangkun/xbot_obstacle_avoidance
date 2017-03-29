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
        self.pub = rospy.Publisher(
            '/cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_dataCB)
        rospy.spin()

    def is_danger(self, scan_data):
        if min(scan_data.ranges[130:230]) < 0.5:
            return True
        return False

    def scan_dataCB(self, scan_data):
        if self.is_danger(scan_data):
            print "obstacle detected"
            self.pub.publish(Twist())

if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbot_safety_stop()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("robot twist node terminated.")
