#!/usr/bin/env python
# coding=utf-8
"""
xbot_EventAction.py
"""
import rospy
from geometry_msgs.msg import Twist

def Advance_Action(xbot_obj):
    pass

def Stop_Action(xbot_obj):
    xbot_obj.pub.publish(Twist())

def Retreat_Action(xbot_obj):
    if len(xbot_obj.navi_deque) > 0:
        xbot_obj.pub.publish(reverse_vel(xbot_obj.navi_deque.pop()))

def A2S_Action(xbot_obj):
    xbot_obj.state = ('Stop', rospy.Time.now())
    Stop_Action(xbot_obj)

def S2A_Action(xbot_obj):
    xbot_obj.state = ('Advance', rospy.Time.now())
    Advance_Action(xbot_obj)

def S2R_Action(xbot_obj):
    xbot_obj.state = ('Retreat', rospy.Time.now())
    Retreat_Action(xbot_obj)

def R2A_Action(xbot_obj):
    xbot_obj.state = ('Advance', rospy.Time.now())
    # publish replan path request
    Advance_Action(xbot_obj)

def A2S_Event(xbot_obj):
    return True if xbot_obj.danger else False

def S2A_Event(xbot_obj):
    return True if not xbot_obj.danger else False

def R2A_Event(xbot_obj):
    return True if not xbot_obj.danger else False

def S2R_Event(xbot_obj):
    stoptime = xbot_obj.state[1]
    return True if rospy.Time.now() - stoptime > xbot_obj.wait_to_retreat_time else False


def define(xbot_obj):
    if not rospy.has_param('~SafeLAng'):
        rospy.set_param('~SafeLAng', 130)
    xbot_obj.SafeLAng = rospy.get_param('~SafeLAng')

    if not rospy.has_param('~SafeRAng'):
        rospy.set_param('~SafeRAng', 230)
    xbot_obj.SafeRAng = rospy.get_param('~SafeRAng')

    if not rospy.has_param('~SafeDistMin'):
        rospy.set_param('~SafeDistMin', 0.3)
    xbot_obj.SafeDistMin = rospy.get_param('~SafeDistMin')

    if not rospy.has_param('~SafeDistMax'):
        rospy.set_param('~SafeDistMax', 0.5)
    xbot_obj.SafeDistMax = rospy.get_param('~SafeDistMax')

    if not rospy.has_param('~RetreatTime'):
        rospy.set_param('~RetreatTime', 4.0)
    xbot_obj.retreat_time = rospy.Duration.from_sec(rospy.get_param('~RetreatTime'))  # the duration of retreat

    if not rospy.has_param('~WaitToRetreatTime'):
        rospy.set_param('~WaitToRetreatTime', 3.0)
    xbot_obj.wait_to_retreat_time = rospy.Duration.from_sec(rospy.get_param('~WaitToRetreatTime'))  # time for waiting retreat command

    print "==========Settings======== "
    print "SafeAng:  [", xbot_obj.SafeLAng, ',', xbot_obj.SafeRAng, ']'
    print "SafeDist:  [", xbot_obj.SafeDistMin, ',', xbot_obj.SafeDistMax, ']'
    print "RetreatTime:  ", xbot_obj.retreat_time.to_sec()
    print "WaitToRetreatTime:  ", xbot_obj.wait_to_retreat_time.to_sec()

def clear(self):
    rospy.delete_param('~SafeLAng')
    rospy.delete_param('~SafeRAng')
    rospy.delete_param('~SafeDistMax')   
    rospy.delete_param('~SafeDistMin')    
    rospy.delete_param('~RetreatTime')
    rospy.delete_param('~WaitToRetreatTime')     


def reverse_vel(vel):
    vel.linear.x = -vel.linear.x 
    vel.linear.y = -vel.linear.y 
    vel.linear.z = -vel.linear.z 
    vel.angular.x = -vel.angular.x 
    vel.angular.y = -vel.angular.y 
    vel.angular.z = -vel.angular.z 
    return vel
