#!/usr/bin/env python3
 
import rospy
from tf import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class Sync_base():
    def __init__(self):
        rospy.init_node('Sync_base', anonymous=True)
        self.br = TransformBroadcaster()
        trans = (0,0,0)
        rot   = (0,0,0,1)
        stamp = rospy.Time.now()
        self.br.sendTransform(trans, rot, stamp,"base","map")

        rospy.Subscriber("/odometry/filtered_map", Odometry, self.base_pose, queue_size=1)
    
    def base_pose(self,Pose):
        self.br = TransformBroadcaster()
        trans = (Pose.pose.pose.position.x, Pose.pose.pose.position.y, Pose.pose.pose.position.z)
        rot   = (Pose.pose.pose.orientation.x, Pose.pose.pose.orientation.y, Pose.pose.pose.orientation.z, Pose.pose.pose.orientation.w)
        self.br.sendTransform(trans, rot, rospy.Time.now(),"base","map") #Pose.header.stamp,"base","map")

if __name__ == '__main__':
    try:
        Sync_base()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
