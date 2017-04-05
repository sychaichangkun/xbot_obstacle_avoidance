#!/usr/bin/env python
# coding=utf-8
"""
xbot_FSM.py
"""


import rospy
import std_msgs.msg
import xbot_EventAction as fsm
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
        print  'current state: ', (statename, time)
        if statename =="Advancing":
            if fsm.A2S_Event(self):       
                fsm.A2S_Action(self)
            else:
                fsm.Advancing_Action(self)
        elif statename == "Stopped":
            if fsm.S2A_Event(self):     
                fsm.S2A_Action(self)
            elif fsm.S2R_Event(self):
                fsm.S2R_Action(self)
            else:
                fsm.Stopped_Action(self)
        elif statename == "Retreating":
            if fsm.R2A_Event(self):                  
                fsm.R2A_Action(self)
            else:
                fsm.Retreating_Action(self)



    def navi_dataCB(self, navi_data):
        if self.state[0] == 'Advancing':
            self.navi_deque.append(navi_data)


    def scan_dataCB(self, scan_data):
        if min(scan_data.ranges[120:240] ) < 0.5:
            self.danger = True
        else:
            self.danger = False



if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbot_FSM()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("xbot_FSM terminated.")
