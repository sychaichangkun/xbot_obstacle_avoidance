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

    states: (statename, time)
    states include 'Advancing', 'Stopped', 'Retreating', #'Waiting'(potential)
    use 'A','S','R' for simplicity

    When specific event is triggered, xbot will transform
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
        (statename, time) = self.state
        print  (statename, time)
        if statename =="Advancing":
            if self.A2S_Event():       #Advancing 2 Stopped Event
                self.A2S_Action()
            else:
                self.Advancing_Action()
        elif statename == "Stopped":
            if self.S2A_Event():     #if not self.danger:
                self.S2A_Action()
            elif self.S2R_Event():
                self.S2R_Action()
            else:
                self.Stopped_Action()
        elif statename == "Retreating":
            if self.R2A_Event():                  #if not self.danger:
                self.R2A_Action()
            else:
                self.Retreating_Action()


    def Advancing_Action(self):
        pass
    def Stopped_Action(self):
        self.pub.publish(Twist())
    def Retreating_Action(self):
        if len(self.navi_deque)>0:
            self.pub.publish(reverse_vel(self.navi_deque.pop()))
    def A2S_Action(self):
        self.state = ('Stopped',rospy.Time.now())
        self.Stopped_Action()
    def S2A_Action(self):
        self.state = ('Advancing',rospy.Time.now())
        self.Advancing_Action()
    def S2R_Action(self):
        self.state = ('Retreating',rospy.Time.now())
        self.Retreating_Action()
    def R2A_Action(self):
        self.state = ('Advancing',rospy.Time.now())
        #publish replan path request
        self.Advancing_Action()
    def A2S_Event(self):
        return True if self.danger else False
    def S2A_Event(self):
        return True if not self.danger else False
    def R2A_Event(self):
         return True if not self.danger else False
    def S2R_Event(self):
        stoppedtime = self.state[1]
        print rospy.Time.now() - stoppedtime
        return True if rospy.Time.now() - stoppedtime > rospy.Duration(2) else False

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
