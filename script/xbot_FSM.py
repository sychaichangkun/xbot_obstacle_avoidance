#!/usr/bin/env python
# coding=utf-8
"""
xbot_FSM.py
"""


import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
import xbot_EventAction as fsm
from sensor_msgs.msg import LaserScan
from collections import deque

inf = 10000
class xbot_FSM():
    """
    xbot Finite-State Machine

    states: (statename, time)
    states include 'Advance', 'Stop', 'Retreat', #'Waiting'(potential)
    use 'A','S','R' for simplicity

    When specific event is triggered, xbot will transform
    from current state to another, with corresponding actions performed
    """

    def __init__(self):
        rospy.init_node('xbot_FSM')
        fsm.define(self)  #define the params in launch file
        self.state = ('Advance', rospy.Time.now())
        self.danger = False                                                 #whether obstacle in safety area or not
        self.navi_deque = deque(maxlen=1000)                #a deque of the path(velocity) just moved   
        self.pub = rospy.Publisher('/cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_dataCB)
        rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.navi_dataCB)
        
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.StateTransition()
            rate.sleep()

    def StateTransition(self):
        (statename, time) = self.state
        print  'current state : ', (statename, time)
        if statename =="Advance":
            if fsm.A2S_Event(self):       
                fsm.A2S_Action(self)
            else:
                fsm.Advance_Action(self)
        elif statename == "Stop":
            if fsm.S2A_Event(self):     
                fsm.S2A_Action(self)
            elif fsm.S2R_Event(self):
                fsm.S2R_Action(self)
            else:
                fsm.Stop_Action(self)
        elif statename == "Retreat":
            if fsm.R2A_Event(self):                  
                fsm.R2A_Action(self)
            else:
                fsm.Retreat_Action(self)

    def navi_dataCB(self, navi_data):
        if self.state[0] == 'Advance':
            self.navi_deque.append(navi_data)

    def scan_dataCB(self, scan_data):
        if min(scan_data.ranges[self.SafeLAng:self.SafeRAng] ) < self.SafeDist:
            self.danger = True
        else:
            self.danger = False


if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbotObj = xbot_FSM()
        fsm.clear(xbotObj)
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("xbot_FSM terminated.")
