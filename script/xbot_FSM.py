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

inf = 10000
class xbot_FSM():
    """
    xbot Finite-State Machine
    States:(action, time)
    Actions:'Advancing', 'Stopped', 'Retreating',
    When specific event is triggered, xbot state will transform 
    from current state to another, with some actions performed
    """

    def __init__(self):
        rospy.init_node('xbot_FSM')
        self.state = ('Advancing', rospy.Time.now())
        print rospy.Time.now()
        self.danger = False
        self.navi_deque = deque(maxlen=1000)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_dataCB)
        rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.navi_dataCB)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.StateTransition()

            rate.sleep()

    def StateTransition(self):
        (action, time) = self.state
        print  (action, time)
        if action =="Advancing":
            if self.danger:
                self.Advancing2Stopped()
            else:
                self.AdvancingAction()
        elif action == "Stopped":
            if not self.danger: 
                self.Stopped2Advancing()
            elif self.StopTimeExceeded(time):
                self.Stopped2Retreating()
            else:
                self.StoppedAction()
        elif action == "Retreating":
            if not self.danger:
                self.Retreating2Advancing()
            else:
                self.RetreatingAction()


    def Advancing2Stopped(self):
        self.state = ('Stopped',rospy.Time.now())
        self.pub.publish(Twist())
    def AdvancingAction(self):
        pass
    def Stopped2Advancing(self):
        self.state = ('Advancing',rospy.Time.now())
        self.AdvancingAction()
    def StopTimeExceeded(self, time):
        print rospy.Time.now() - time
        return True if rospy.Time.now() - time > rospy.Duration(2) else False
    def Stopped2Retreating(self):
        self.state = ('Retreating',rospy.Time.now())
        self.RetreatingAction()
    def StoppedAction(self):
        self.pub.publish(Twist())
    def Retreating2Advancing(self):
        self.state = ('Advancing',rospy.Time.now())
        #publish replan path request
        self.AdvancingAction()
    def RetreatingAction(self):
        if len(self.navi_deque)>0:
            self.pub.publish(reverse_vel(self.navi_deque.pop()))


    def navi_dataCB(self, navi_data):
        if self.state[0] == 'Advancing':
            self.navi_deque.append(navi_data)


    def scan_dataCB(self, scan_data):
        if min(scan_data.ranges[120:240] ) < 0.5:
            self.danger = True
        else:
            self.danger = False

def reverse_vel(vel):
    vel.linear.x = -vel.linear.x 
    vel.linear.y = -vel.linear.y 
    vel.linear.z = -vel.linear.z 
    vel.angular.x = -vel.angular.x 
    vel.angular.y = -vel.angular.y 
    vel.angular.z = -vel.angular.z 
    return vel



if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbot_FSM()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("xbot_FSM terminated.")
