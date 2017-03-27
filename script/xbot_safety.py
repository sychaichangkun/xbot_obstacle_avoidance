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
        self.retreat_time = rospy.Duration(1, 0)  # the duration of retreat
        self.wait_to_retreat_time = rospy.Duration(0, 0)  # time for waiting retreat command
        self.navi_deque = deque(maxlen=10000) #deque to store navi_vel
        self.stopped = False #Flag
        self.retreating = False #Flag
        self.pub = rospy.Publisher(
            '/cmd_vel_mux/input/safety_controller', Twist, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_dataCB)
        rospy.Subscriber("/cmd_vel_mux/input/navi", Twist, self.navi_dataCB)
        # rospy.Subscriber("/cmd_retreat", std_msgs.msg.Int32, self.retreat_dataCB)
        #rospy.spin()
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            rate.sleep()


    def scan_dataCB(self, scan_data):
        if min(scan_data.ranges[120:240] ) < 0.5:      
            if not self.stopped:  #if the first time to stop
                self.wait_start_time = rospy.Time.now() 
                print "turn to stopped, start to count time "
            elif rospy.Time.now() - self.wait_start_time > self.wait_to_retreat_time:
                print "begin to retreat"
                # retreat func:
                self.retreat()
                self.stopped = False
            else:
                print "stopped"
                cmd = Twist()  # stop the robot
                cmd.linear.x = 0
                cmd.angular.z = 0
                self.pub.publish(cmd)
            self.stopped = True
        else:
            print 'move'
            self.stopped = False


    def retreat(self):
        rospy.loginfo("start to retreat")
        if len(self.navi_deque) > 1:
            navi_data_now = self.navi_deque.pop()
        retreat_start_time = rospy.Time.now() 
        while len(self.navi_deque) > 0 and rospy.Time.now()  - retreat_start_time < self.retreat_time:
            print rospy.Time.now()  - retreat_start_time - self.retreat_time
            #print "navi_deque len is ", len(self.navi_deque)
            navi_data_prev = self.navi_deque.pop()
            delta_time = navi_data_now[1] - navi_data_prev[1]
            #rospy.loginfo("delta time  %i %i", delta_time.secs, delta_time.nsecs)
            #rospy.sleep(rospy.Duration(delta_secs, delta_nsecs))
            rospy.sleep(rospy.Duration(delta_time.secs, delta_time.nsecs))
            #self.pub.publish(navi_data_now[0])
            self.pub.publish(reverse_vel(navi_data_now[0]))
            navi_data_now = navi_data_prev
        self.navi_deque.clear()
        rospy.loginfo("end retreat")


    def navi_dataCB(self, navi_data):
        if not self.stopped:
            # navi_deque consists  (navi_data, time)
            #rospy.loginfo("navi_data append")
            self.navi_deque.append((navi_data, rospy.Time.now()))


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

