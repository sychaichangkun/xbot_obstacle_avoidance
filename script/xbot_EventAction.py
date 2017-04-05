#!/usr/bin/env python
# coding=utf-8
"""
xbot_EventAction.py
"""
import rospy
from geometry_msgs.msg import Twist

def Advancing_Action(xbot_obj):
    pass

def Stopped_Action(xbot_obj):
    xbot_obj.pub.publish(Twist())

def Retreating_Action(xbot_obj):
    if len(xbot_obj.navi_deque) > 0:
        xbot_obj.pub.publish(reverse_vel(xbot_obj.navi_deque.pop()))

def A2S_Action(xbot_obj):
    xbot_obj.state = ('Stopped', rospy.Time.now())
    xbot_obj.Stopped_Action()

def S2A_Action(xbot_obj):
    xbot_obj.state = ('Advancing', rospy.Time.now())
    xbot_obj.Advancing_Action()

def S2R_Action(xbot_obj):
    xbot_obj.state = ('Retreating', rospy.Time.now())
    xbot_obj.Retreating_Action()

def R2A_Action(xbot_obj):
    xbot_obj.state = ('Advancing', rospy.Time.now())
    # publish replan path request
    xbot_obj.Advancing_Action()

def A2S_Event(xbot_obj):
    return True if xbot_obj.danger else False

def S2A_Event(xbot_obj):
    return True if not xbot_obj.danger else False

def R2A_Event(xbot_obj):
    return True if not xbot_obj.danger else False

def S2R_Event(xbot_obj):
    stoppedtime = xbot_obj.state[1]
    print rospy.Time.now() - stoppedtime
    return True if rospy.Time.now() - stoppedtime > rospy.Duration(2) else False





def reverse_vel(vel):
    vel.linear.x = -vel.linear.x 
    vel.linear.y = -vel.linear.y 
    vel.linear.z = -vel.linear.z 
    vel.angular.x = -vel.angular.x 
    vel.angular.y = -vel.angular.y 
    vel.angular.z = -vel.angular.z 
    return vel
