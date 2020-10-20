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
        self.distance_new = 1
        self.distance_threshold = 1 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub_Waypoints = rospy.Publisher('/Current_Waypoints', PoseStamped, queue_size=1)
        self.pub_goal = rospy.Publisher('/Current_goal', PoseStamped, queue_size=1)

        rospy.Subscriber("/Published_pose",PoseStamped,self.New_input, queue_size=1)
        rospy.Subscriber("/Vehicle_pose",PoseStamped,self.Compare_pose,queue_size=1)
        
    def New_input(self,Pose):
        print("A New input have been received")
        if self.Waypoints == []:
            self.Waypoints.append(Pose)
            #print("There are no waypoints to follow")
        else: 
            #cwp = self.Waypoints[0]
            cwp = Pose
            self.tf.waitForTransform("map", Pose.header.frame_id, Pose.header.stamp, rospy.Duration(1.0))
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
            
            # cwdistance is distance between the latest added point and the newly found point
            cwdistance = math.sqrt((cwp.pose.position.x-self.Waypoints[-1].pose.position.x)**2+(cwp.pose.position.y-self.Waypoints[-1].pose.position.y)**2)
            if cwdistance > self.distance_new:
                self.Waypoints.append(cwp)

        self.pub_Waypoints.publish(self.Waypoints)

    def Compare_pose(self,Pose):
        print("Comparing vehicle pose with current goal")
        if distance2waypoint < self.distance_threshold:
            self.Waypoints.pop(0)
            if self.Waypoints = []:
                print("Arrived at distination with safe distance.")
            else:
                self.pub_goal.publish(self.Waypoints[0])
                print("New Waypoint given")

        




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