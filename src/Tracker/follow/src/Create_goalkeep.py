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
        rospy.init_node('Create_goal')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.Waypoints = []
        self.Move_base_goal = []
        self.Waypoint = Pose()
        self.Waypoints = PoseArray()
        self.Waypoints_reached = PoseArray()

        self.tf = TransformListener()
        # Tolerances
        self.distance_keep = 1 #[m]
        self.distance_new = 1.5
        self.distance_threshold = 0.1 # Distance to a waypoint before it is discarded
        self.twist_threshold = 30*math.pi/180

        # Subscribed topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_waypoint_list = rospy.Publisher("/Tracker/Waypoint_list",PoseArray,queue_size=1)
        self.pub_waypoint_list_reached = rospy.Publisher("/Tracker/Waypoint_list_deleted",PoseArray,queue_size=1)
        print("Waiting for Odometry...")
        try:
            self.Current_position = rospy.wait_for_message("/odometry/filtered_map",Odometry,timeout=5)
            print("Odometry received")
        except:
            self.Current_position = Odometry()
            self.Current_position.pose.pose.position.x = 0
            self.Current_position.pose.pose.position.y = 0
            self.Current_position.pose.pose.position.z = 0
            self.Current_position.pose.pose.orientation.w = 1
            self.Current_position.pose.pose.orientation.x = 0
            self.Current_position.pose.pose.orientation.y = 0
            self.Current_position.pose.pose.orientation.z = 0
            print("Odometry created")
        
        self.Move_base_goal = self.Current_position.pose
        
        #rospy.Subscriber("/odometry/filtered_map",Odometry,self.Compare_pose,queue_size=1)
        rospy.Subscriber("/Tracker/Object_Tracker/Published_pose",PoseStamped,self.New_input, queue_size=1)
        rospy.Subscriber("/move_base/Current_goal",PoseStamped,self.Current_goal, queue_size=1)
        
        
    def New_input(self,Pose):
        # This callback is responsible of creating a list of goals that the vehicle is to follow in order
        self.Waypoints.header = Pose.header
        Waypoint = Pose.pose

        # Calculate Transforms and the Poses in their respective frames
        # Transform from "Pose Frame" to base_link
        #transform_pb = self.tf_buffer.lookup_transform("base_link", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        # Transform from "Pose Frame" to map
        transform_pm = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        # Pose in base_link frame
        #np_b = tf2_geometry_msgs.do_transform_pose(Pose,transform_pb)
        # Pose in map frame
        np_m = tf2_geometry_msgs.do_transform_pose(Pose,transform_pm)
        # Mapping only takes zero values in z
        #np_b.pose.position.z = 0
        np_m.pose.position.z = 0 


        if len(self.Waypoints.poses) == 0:
            # If there is no point add a point at the current goal (Only first loop will run this part)
            vec_new_old=np.array([np_m.pose.position.x,np_m.pose.position.y,np_m.pose.position.z])
            quaternion=get_new_orientation(self.Move_base_goal,np_m,0,Points=True)
            Waypoint.position.x = self.Move_base_goal.pose.position.x
            Waypoint.position.y = self.Move_base_goal.pose.position.y
            Waypoint.position.z = self.Move_base_goal.pose.position.z
            Waypoint.orientation.x = quaternion[0]
            Waypoint.orientation.y = quaternion[1]
            Waypoint.orientation.z = quaternion[2]
            Waypoint.orientation.w = quaternion[3]
            self.Waypoints.poses.append(Waypoint)

        distance_to_object = math.sqrt((np_m.pose.position.x-self.Current_position.pose.pose.position.x)**2+(np_m.pose.position.y-self.Current_position.pose.pose.position.y)**2)


        lp_m = PoseStamped()
        lp_m.pose = self.Waypoints.poses[-1]
        Waypoint = np_m.pose
        quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
        Waypoint.position.x = Waypoint.position.x
        Waypoint.position.y = Waypoint.position.y
        Waypoint.position.z = Waypoint.position.z
        Waypoint.orientation.x = quaternion[0]
        Waypoint.orientation.y = quaternion[1]
        Waypoint.orientation.z = quaternion[2]
        Waypoint.orientation.w = quaternion[3]
        self.Waypoints.poses.append(Waypoint)

        distance_between_latest_points = math.sqrt((self.Waypoints.poses[-2].position.x-self.Waypoints.poses[-1].position.x)**2+(self.Waypoints.poses[-2].position.y-self.Waypoints.poses[-1].position.y)**2)

        if distance_between_latest_points <= self.distance_keep:
            # If within distance keep distance, update the orientation on both points
            self.Waypoints.poses[-2].orientation.x = quaternion[0]
            self.Waypoints.poses[-2].orientation.y = quaternion[1]
            self.Waypoints.poses[-2].orientation.z = quaternion[2]
            self.Waypoints.poses[-2].orientation.w = quaternion[3]
            self.pub_waypoint_list.publish(self.Waypoints)
            del self.Waypoints.poses[-1]

    
    def Compare_pose(self,Pose):
        self.Current_position = Pose

        Waypoint = PoseStamped()
        Waypoint.header = Pose.header
        self.Waypoints_reached.header = Pose.header

        # Check if there is a current goal
        if self.Move_base_goal != []:
            pass
                

    def Current_goal(self,Pose):
        self.Move_base_goal = Pose

    def update_waypoints(self,Pose):
        self.Waypoints2.header = Pose.header
        self.Waypoints2.poses.append(Pose.pose)
        self.pub_waypoint_list.publish(self.Waypoints2)



def temp(Pose, xy):
    Pose.pose.position.x=xy[0] # Udkommenter hvis den skal køre ind i object
    Pose.pose.position.y=xy[1] # Udkommenter hvis den skal køre ind i object
    temp = Pose
    return temp

if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass