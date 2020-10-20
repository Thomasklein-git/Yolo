#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
import math
from pyquaternion import Quaternion

class Follow():
    def __init__(self):
        rospy.init_node('Poser', anonymous=True)

        self.Waypoints = []

        self.tf = TransformListener()
        # Tolerances
        #self.distance_keep = 1 #[m]
        #self.distance_threshold = 1 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub = rospy.Publisher('/Current_pose', PoseStamped, queue_size=1)

        rospy.Subscriber("/Published_pose",PoseStamped,self.New_input, queue_size=1)
        rospy.Subscriber("/Vehicle_pose",PoseStamped,self.Compare_pose,queue_size=1)
        
        
    def New_input(self,Pose):
        print("A New input have been received")
        if self.Waypoints == []:
            #print("There are no waypoints to follow")
            pass
        else: 
            #cwp = self.Waypoints[0]
            cwp = Pose
            trans,rot = self.tf.lookupTransform("map", Pose.header.frame_id, Pose.header.stamp)
            cwp.header.frame_id = "map"
            cwp.pose.position.x += trans[0]#Pose.pose.position.x+trans[0]
            cwp.pose.position.y += trans[1]
            cwp.pose.position.z += trans[2]
            cwpq = Quaternion(cwp.pose.orientation.w,cwp.pose.orientation.x,cwp.pose.orientation.y,cwp.pose.orientation.z) # CurrentWayPointQuerternion
            mapq = Quaternion([rot[3],rot[0],rot[1],rot[2]]) # Map2CurrentPoseQuerternion
            q = cwpq*mapq
            cwp.pose.orientation.x = q[1]
            cwp.pose.orientation.y = q[2]
            cwp.pose.orientation.z = q[3]
            cwp.pose.orientation.w = q[0]
            
            #print( Pose.y+trans[1])
        #dfromnp2cwp = 
        #if distance2newpoint < self.distance_threshold:
        self.Waypoints.append(Pose)
        #print(self.Waypoints)

        # Distance to 

        self.pub.publish(self.Waypoints[0])
        #rospy.sleep(2)
    

    def Compare_pose(self,Pose):
        print("Comparing vehicle pose with current goal")




    #def Current_Waypoint(self,pose):
        #Current_pose = pose #The current incoming pose


        #self.Waypoints.append(Current_pose)
    #
    #def Remove_Waypoint(self,pose)
        # Removes the current point when within the distance threshold
    #    if distance2waypoint < self.distance_threshold
    #        self.Waypoints

    #def Waypoints():
    #    self.Waypoints
if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass