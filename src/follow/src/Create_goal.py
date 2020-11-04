#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf import TransformListener
import math
from agfh import *

class Follow():
    def __init__(self):
        rospy.init_node('Poser', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.Waypoints = []
        self.Move_base_goal = []

        self.tf = TransformListener()
        # Tolerances
        self.distance_keep = 0.5 #[m]
        self.distance_new = 1
        self.distance_threshold = 0.5 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        rospy.Subscriber("/odometry/filtered_map",PoseStamped,self.Compare_pose,queue_size=1)
        rospy.Subscriber("/Published_pose",PoseStamped,self.New_input, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.Current_goal, queue_size=1)
        
        
    def New_input(self,Pose):
        Waypoints = self.Waypoints
        transform = self.tf_buffer.lookup_transform("base_link", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        np_b = tf2_geometry_msgs.do_transform_pose(Pose, transform) # New Waypoint in base
        npd = math.sqrt((np_b.pose.position.x)**2+(np_b.pose.position.y)**2)

        # Check if point from Zed cam is further away than distance_threshold
        if npd > self.distance_threshold:
            transform_np = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            np_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_np)
            np_m.pose.position.z = 0

            # If the waypoint is further away than distance_threshold and Waypoint list is empty add point to waypoints
            # If list is not empty compare new waypoint with the latest added waypoint to ensure that distance between the two points are greater than distance_new
            if Waypoints == []:
                vec_new_old=np.array([np_b.pose.position.x,np_b.pose.position.y,np_b.pose.position.z])
                quaternion=get_new_orientation(0,0,vec_new_old,Points=False)
                np_m.pose.orientation.x=quaternion[0]
                np_m.pose.orientation.y=quaternion[1]
                np_m.pose.orientation.z=quaternion[2]
                np_m.pose.orientation.w=quaternion[3]
                #print(np_m,"empty")
                self.Waypoints.append(np_m)
            else: 
                lp_m = Waypoints[-1]
                quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                np_m.pose.orientation.x=quaternion[0]
                np_m.pose.orientation.y=quaternion[1]
                np_m.pose.orientation.z=quaternion[2]
                np_m.pose.orientation.w=quaternion[3]
                #print(np_m,"not empty")
                wpd = math.sqrt((np_m.pose.position.x-lp_m.pose.position.x)**2+(np_m.pose.position.y-lp_m.pose.position.y)**2)
                if wpd > self.distance_new:
                    self.Waypoints.append(np_m)

    def Compare_pose(self,Pose):
        # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
        # If waypoint list is empty do nothing
        if len(self.Waypoints) == 0:
            pass
        # If waypoint list contains points 
        else:
            # Calculate the distance to all current waiting waypoints
            d2pb = []
            transform_bm = self.tf_buffer.lookup_transform("map","base_link", Pose.header.stamp, rospy.Duration(1.0))
            transform_mb = self.tf_buffer.lookup_transform("base_link","map", Pose.header.stamp, rospy.Duration(1.0))
            for Point_m in self.Waypoints:
                Point_b = tf2_geometry_msgs.do_transform_pose(Point_m, transform_mb)
                d2pb.append(math.sqrt(Point_b.pose.position.x**2+Point_b.pose.position.y**2))
            minpos = d2pb.index(min(d2pb))
            # If any waypoints in the list is closer to the base, discard all waypoints up to the closest point.
            if minpos > 0:
                for i in range(0,minpos):
                    del self.Waypoints[0]


            # Check if the current Waypoint in 
            Goal_m      = self.Waypoints[0]
            Goal_b    = tf2_geometry_msgs.do_transform_pose(Goal_m, transform_mb)
            print(Goal_b)
            distance2waypoint = math.sqrt(Goal_b.pose.position.x**2+Goal_b.pose.position.y**2)
            print(distance2waypoint)
            # if the waypoint is further away than distance threshold, check if point is the current goal or else publish as current goal
            if distance2waypoint > self.distance_threshold:
                # If current waypoint is the same as goal, pass
                if self.Move_base_goal == self.Waypoints[0]:
                    pass
                # If current waypoint is not same as goal, publish current waypoint and add current goal
                else:
                    self.pub_goal.publish(self.Waypoints[0])
                    #self.Move_base_goal = self.Waypoints[0]:

            # If the waypoint is within distance threshold remove current waypoint               
            else:
                del self.Waypoints[0]
                # If list is now empty do nothing
                if len(self.Waypoints) == 0:
                    pass
                # If list containt more poses, publish a new goal
                else:
                    self.pub_goal.publish(self.Waypoints[0])
                    #self.Move_base_goal = self.Waypoints[0]

    def Current_goal(self,Pose):
        self.Move_base_goal = Pose

if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass