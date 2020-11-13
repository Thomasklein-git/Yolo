#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
import random
import time

class Keyboard_pose:
    def __init__(self):
        rospy.init_node("Keyboard_Move")
        self.Pose = PoseStamped()
        self.pub = rospy.Publisher('/Tracker/Object_Tracker/Published_pose', PoseStamped, queue_size=1)
        rospy.Subscriber("/Keyboard/Twist",Twist,self.cb_Twist,queue_size=1)
        

    def cb_Twist(self,Twist):
        self.Pose.header.stamp = rospy.Time.now()
        self.Pose.header.frame_id = "map"
        self.Pose.pose.position.x += Twist.linear.x
        self.Pose.pose.position.y += Twist.linear.y
        self.Pose.pose.position.z += Twist.linear.z

        self.Pose.pose.orientation.x = 0
        self.Pose.pose.orientation.y = 0
        self.Pose.pose.orientation.z = 0
        self.Pose.pose.orientation.w = 1

        self.pub.publish(self.Pose)
         
if __name__ == '__main__':
    try:
        Keyboard_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass