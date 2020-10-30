#!/usr/bin/env python3
 
import rospy
from tf import TransformBroadcaster
from geometry_msgs.msg import PoseStamped

class Sync_base():
    def __init__(self):
        rospy.init_node('Sync_base', anonymous=True)
        self.br = TransformBroadcaster()
        trans = (0,0,0)
        rot   = (0,0,0,1)
        stamp = rospy.Time.now()
        self.br.sendTransform(trans, rot, stamp,"base","map")

        rospy.Subscriber("/odometry/filtered_map", PoseStamped, self.base_pose, queue_size=1)
    
    def base_pose(self,Pose):
        self.br = TransformBroadcaster()
        trans = (Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z)
        rot   = (Pose.pose.orientation.x, Pose.pose.orientation.y, Pose.pose.orientation.z, Pose.pose.orientation.w)
        self.br.sendTransform(trans, rot, rospy.Time.now(),"base","map") #Pose.header.stamp,"base","map")

if __name__ == '__main__':
    try:
        Sync_base()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
