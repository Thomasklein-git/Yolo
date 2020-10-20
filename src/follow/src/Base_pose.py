#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf import TransformBroadcaster
import math

class Base_pose():
    def __init__(self):
        rospy.init_node('Base_position', anonymous=True)
        self.Current_goal
        self.Movedir = []
        self.VPose = PoseStamped()
        self.VPose.header.stamp = rospy.Time.now()
        self.VPose.header.frame_id = "/map"
        self.VPose.pose.position.x    = float(0)
        self.VPose.pose.position.y    = float(0)
        self.VPose.pose.position.z    = float(0)
        self.VPose.pose.orientation.x = float(0)
        self.VPose.pose.orientation.y = float(0)
        self.VPose.pose.orientation.z = float(0)
        self.VPose.pose.orientation.w = float(1)

        rate = rospy.Rate(1) # 1hz

    def Current_goal(self,Pose):
        self.Current_goal = Pose
    def Current_goal(self,Pose):
        print("CG")
        if self.Current_goal == []:
            self.Movedir = []
        else:
            Movemag = math.sqrt((Pose.pose.position.x-self.VPose.pose.position.x)**2+(Pose.pose.position.y-self.VPose.pose.position.y)**2+(Pose.pose.position.z-self.VPose.pose.position.z)**2)
        self.Movedir = [Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z]/Movemag*1
        self.VPose.pose.position.x += self.Movedir[0]
        self.VPose.pose.position.x += self.Movedir[1]
        self.VPose.pose.position.x += self.Movedir[2]
        self.base_pub = rospy.Publisher('/Vehicle_pose', PoseStamped, queue_size=1)

if __name__ == '__main__':
    try:
        Base_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass