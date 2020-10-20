#!/usr/bin/env python3
 
import rospy
from tf import TransformBroadcaster
from geometry_msgs.msg import PoseStamped

def base_pose(Pose):
    br = TransformBroadcaster()
    trans = (Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z)
    rot   = (Pose.pose.orientation.x, Pose.pose.orientation.y, Pose.pose.orientation.z, Pose.pose.orientation.w)
    br.sendTransform(trans, rot, Pose.header.stamp,"BAM","map")

if __name__ == '__main__':
    rospy.init_node('Base_broadcaster')
    rospy.Subscriber("/Published_pose", PoseStamped, base_pose, queue_size=1)
    rospy.spin()
