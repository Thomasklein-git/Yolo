#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
import tf2_ros
import tf2_geometry_msgs
from tf import TransformListener
import math


import os 
import sys
dir_to_Tracker=os.path.dirname(os.path.dirname(os.path.dirname( __file__ )))
dir_to_Scripts = os.path.join(dir_to_Tracker,"Scripts") 
sys.path.append(dir_to_Scripts)
from agfh import *

class Follow():
    def __init__(self):
        rospy.init_node('Offset_goal')
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Tolerances
        self.distance_left   = 0.6
        self.distance_right  = 0.6
        self.distance_behind = 1.2

        # Subscribed topic
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        rospy.Subscriber("/Tracker/move_base_goal",PoseStamped,self.callback_offset,queue_size=1)

    def callback_offset(self,Pose):
        """
        Formations:                  Target
            0: Front                   |
            1: Left                  Front
            2: Right                   |
            3: Behind           Left -- -- Right
        Follows the Target             |
        at specified distance        Behind 
        """
        if rospy.has_param("/Formation"):
            Formation = rospy.get_param("/Formation")
        else:
            Formation = 0

        if Formation == 0:
            displacement = [0,0,0]
        elif Formation == 1:
            displacement = [-self.distance_left,0.3,0]            
        elif Formation == 2:
            displacement = [-self.distance_right,-0.3,0]
        elif Formation == 3:
            displacement = [-self.distance_behind,0,0]

        quaternion   = Quaternion(Pose.pose.orientation.w,Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z)
        displacement = quaternion.rotate(displacement)
        Pose.pose.position.x += displacement[0]
        Pose.pose.position.y += displacement[1]
        Pose.pose.position.z += displacement[2]
        self.pub_goal.publish(Pose)


if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass