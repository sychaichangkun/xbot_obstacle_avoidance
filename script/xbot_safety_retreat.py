#!/usr/bin/env python
# coding=utf-8
"""
xbot_safety.py
"""

import rospy
import std_msgs.msg
from geometry_msgs.msg import Twist
from xbot_msgs.msg import DockInfraRed
from sensor_msgs.msg import LaserScan
from collections import deque

inf = 1000


def reverse_vel(vel):
    vel.linear.x = -vel.linear.x 
    vel.linear.y = -vel.linear.y 
    vel.linear.z = -vel.linear.z 
    vel.angular.x = -vel.angular.x 
    vel.angular.y = -vel.angular.y 
    vel.angular.z = -vel.angular.z 
    return vel



class xbot_retreat():
    """
    stop xbot when  echo data received
    retreat when xbot keep stopped for more than wait_to_retreat_time
    """

    def __init__(self):
        rospy.init_node('xbot_retreat')
        self.pub = rospy.Publisher('/cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        self.pub2 = rospy.Publisher('/replan_path', std_msgs.msg.Bool, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_dataCB)
        rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.navi_dataCB)
        # rospy.Subscriber("/cmd_retreat", std_msgs.msg.Int32, self.retreat_dataCB)
        #rospy.spin()
        self.retreat_time = rospy.Duration(3, 0)  # the duration of retreat
        self.wait_to_retreat_time = rospy.Duration(2, 0)  # time for waiting retreat command
        self.navi_deque = deque(maxlen=10000) #deque to store navi_vel
        self.stopped = False #Flag
        self.retreated = False #Flag   
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            rate.sleep()

    def is_danger(self,scan_data):
        if min(scan_data.ranges[120:240] ) < 0.5:return True;
        return False;

    def scan_dataCB(self, scan_data):
        print "scan_dataCB, stopped:",self.stopped
        if self.is_danger(scan_data):      
            if not self.stopped:  #if the first time to stop
                self.wait_start_time = rospy.Time.now() 
                print "turn to stopped, start to count time "
                self.stopped = True
            elif rospy.Time.now() - self.wait_start_time > self.wait_to_retreat_time:
                print "begin to retreat"
                print "Time.now() ", rospy.Time.now() 
                print "wait_start_time", self.wait_start_time 
                self.retreat()
                self.stopped = False
            else:
                print "stopped"
                self.pub.publish(Twist())
                self.stopped = True
        else:
            if self.retreated == True:#when xbot just retreated, request navigation to replan path
                self.pub2.publish(True)
            print 'move'
            self.retreated = False
            self.stopped = False


    def retreat(self):
        self.retreated = True
        self.stopped = False
        retreat_start_time = rospy.Time.now() 
        r = rospy.Rate(50) 
        while len(self.navi_deque) > 0 and rospy.Time.now()  - retreat_start_time < self.retreat_time:
            print "len(navi_deque)",len(self.navi_deque)
            print "time",self.retreat_time- (rospy.Time.now()  - retreat_start_time)
            self.pub.publish(reverse_vel(self.navi_deque.pop()))
            r.sleep()
        #self.navi_deque.clear()
        rospy.loginfo("end retreat")


    def navi_dataCB(self, navi_data):
        if not self.stopped:
            self.navi_deque.append(navi_data)


    # def retreat_dataCB(self, retreat):
    #     self.stopped = True
        # rospy.loginfo("start to retreat")
        # if len(self.navi_deque) > 1: navi_data_now = self.navi_deque.pop()
        # retreat_start_time = rospy.get_time()
        # while len(self.navi_deque) > 0 and rospy.get_time() - retreat_start_time < self.retreat_time:
        #     print "rospy.get_time() - retreat_start_time", rospy.get_time() - retreat_start_time
        #     print "navi_deque len is ", len(self.navi_deque)
        #     navi_data_prev = self.navi_deque.pop()
        #     delta_secs =  navi_data_now[1].secs - navi_data_prev[1].secs
        #     delta_nsecs =  navi_data_now[1].nsecs - navi_data_prev[1].nsecs
        #     rospy.loginfo("delta time  %i %i", delta_secs , delta_nsecs)
        #     rospy.sleep(rospy.Duration(delta_secs, delta_nsecs))
        #     self.pub.publish(navi_data_now[0])
        #     navi_data_now = navi_data_prev
        # rospy.loginfo("end retreat")
        # self.stopped = False

if __name__ == '__main__':
    try:
        rospy.loginfo("initialization system")
        xbot_retreat()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("xbot_retreat terminated.")

