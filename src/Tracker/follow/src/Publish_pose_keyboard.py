#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from pyquaternion import Quaternion
import numpy as np
import random
import time

class Keyboard_pose:
    def __init__(self):
        rospy.init_node("Keyboard_Move")
        self.Pose = PoseStamped()
        self.Pose.pose.orientation.x = 0
        self.Pose.pose.orientation.y = 0
        self.Pose.pose.orientation.z = 0
        self.Pose.pose.orientation.w = 1
        self.pub = rospy.Publisher('/Tracker/Object_Tracker/Published_pose', PoseStamped, queue_size=1)
        self.vis = rospy.Publisher('/Tracker/Visualization/Published_pose', PoseStamped, queue_size=1)
        rospy.Subscriber("/Keyboard/Twist",Twist,self.cb_Twist,queue_size=1)
        self.translation_step = 0.1
        self.rotation_step    = 0.1
        

    def cb_Twist(self,Twist):
        Twist.angular.x = Twist.angular.z*self.rotation_step
        Twist.angular.y = Twist.angular.z*self.rotation_step
        Twist.angular.z = Twist.angular.z*self.rotation_step

        q1 = Quaternion(axis=[0,0,1], angle=Twist.angular.z)
        q2 = Quaternion(self.Pose.pose.orientation.w,self.Pose.pose.orientation.x,self.Pose.pose.orientation.y,self.Pose.pose.orientation.z)
        q3 = q2*q1

        self.Pose.pose.orientation.x = q3[1]
        self.Pose.pose.orientation.y = q3[2]
        self.Pose.pose.orientation.z = q3[3]
        self.Pose.pose.orientation.w = q3[0]

        Twistvec    = np.array([Twist.linear.x,Twist.linear.y,Twist.linear.z])        

        Twistrotvec = q3.rotate(Twistvec)

        self.Pose.header.stamp = rospy.Time.now()
        self.Pose.header.frame_id = "map"
        self.Pose.pose.position.x += Twistrotvec[0]*self.translation_step
        self.Pose.pose.position.y += Twistrotvec[1]*self.translation_step
        self.Pose.pose.position.z += Twistrotvec[2]*self.translation_step
        """
        self.Pose.pose.orientation.x = 0
        self.Pose.pose.orientation.y = 0
        self.Pose.pose.orientation.z = 0
        self.Pose.pose.orientation.w = 1
        """
        pubState = rospy.get_param("/poseState")
        if pubState == "Publish":
            self.vis.publish(self.Pose)
            self.pub.publish(self.Pose)
        elif pubState == "Silent":
            self.vis.publish(self.Pose)
         
if __name__ == '__main__':
    try:
        Keyboard_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass