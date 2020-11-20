#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class follow_rviz:
    def __init__(self):
        rospy.init_node('Create_Markers')
        rospy.Subscriber("/odometry/filtered_map",Odometry,self.cb_odom,queue_size=1)
        rospy.Subscriber("/Tracker/Object_Tracker/Published_pose",PoseStamped,self.cb_pose, queue_size=1)
        self.pub_odom_marker = rospy.Publisher("/Tracker/Visualization/Vehicle_position",Marker,queue_size=1)
        self.pub_pose_marker = rospy.Publisher("/Tracker/Visualization/Object_position",Marker,queue_size=1)
        rospy

    def cb_odom(self,Odometry):
        Mark = Marker()
        Mark.header = Odometry.header
        Mark.pose = Odometry.pose.pose
        Mark.id = 0
        Mark.type = 1
        Mark.action = 0

        Mark.scale.x = 0.1
        Mark.scale.y = 0.1
        Mark.scale.z = 0.1

        Mark.color.r = 1
        Mark.color.b = 0
        Mark.color.g = 0
        Mark.color.a = 1

        #Mark.lifetime = 0
        Mark.frame_locked = False
        self.pub_odom_marker.publish(Mark)
    
    def cb_pose(self,Pose):
        Mark = Marker()
        Mark.header = Pose.header
        Mark.pose = Pose.pose
        Mark.id = 1
        Mark.type = 1
        Mark.action = 0

        Mark.scale.x = 0.1
        Mark.scale.y = 0.1
        Mark.scale.z = 0.1

        Mark.color.r = 0
        Mark.color.b = 1
        Mark.color.g = 0
        Mark.color.a = 1

        #Mark.lifetime = 0
        Mark.frame_locked = False
        self.pub_pose_marker.publish(Mark)




if __name__ == '__main__':
    try:
        follow_rviz()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass