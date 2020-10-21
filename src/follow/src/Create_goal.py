#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf import TransformListener
import math
from pyquaternion import Quaternion

class Follow():
    def __init__(self):
        rospy.init_node('Poser', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.Waypoints = []

        self.tf = TransformListener()
        # Tolerances
        #self.distance_keep = 1 #[m]
        self.distance_new = 0.5
        self.distance_threshold = 1 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub_Waypoints = rospy.Publisher('/Current_Waypoints', PoseStamped, queue_size=1)
        self.pub_goal = rospy.Publisher('/Current_goal', PoseStamped, queue_size=1)

        rospy.Subscriber("/Published_pose",PoseStamped,self.New_input, queue_size=1)
        rospy.Subscriber("/Vehicle_pose",PoseStamped,self.Compare_pose,queue_size=1)
        
    def New_input(self,Pose):
        Waypoints = self.Waypoints
        transform = self.tf_buffer.lookup_transform("base", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        np_b = tf2_geometry_msgs.do_transform_pose(Pose, transform) # New point in base
        npd = math.sqrt((np_b.pose.position.x)**2+(np_b.pose.position.y)**2)
        if npd > self.distance_threshold:

            transform_np = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            np_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_np)

            if Waypoints == []:
                self.Waypoints.append(np_m)
                self.pub_goal.publish(self.Waypoints[0])
                print("Type 1")
            else: 
                transform_np   = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                transform_lp   = self.tf_buffer.lookup_transform("map", Waypoints[-1].header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                
                np_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_np) # New Waypoint in map
                lp_m = tf2_geometry_msgs.do_transform_pose(Waypoints[-1], transform_lp) # Last Waypoint in map
                wpd = math.sqrt((np_m.pose.position.x-lp_m.pose.position.x)**2+(np_m.pose.position.y-lp_m.pose.position.y)**2)

                if wpd > self.distance_new:
                    self.Waypoints.append(np_m)

        """
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
        """
        #self.pub_Waypoints.publish(self.Waypoints)

    def Compare_pose(self,Pose):
        if self.Waypoints == []:
            print("Waiting for a purpose")
        else:
            print("Comparing vehicle pose with current goal")
            # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
            Goal      = self.Waypoints[0]
            print(Goal)
            transform = self.tf_buffer.lookup_transform("base",Goal.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            Goal_b    = tf2_geometry_msgs.do_transform_pose(Goal, transform)
            print(Goal_b)
            distance2waypoint = math.sqrt(Goal_b.pose.position.x**2+Goal_b.pose.position.y**2)
            print(distance2waypoint,"Dist")
            if distance2waypoint < self.distance_threshold:
                self.Waypoints.pop(0)
                if self.Waypoints == []:
                    print("Arrived at distination with safe distance.")
                else:
                    self.pub_goal.publish(self.Waypoints[0])
                    print(self.Waypoints[0], "Type 3")
                    print("New Waypoint given")

if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass