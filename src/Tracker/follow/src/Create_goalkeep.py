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
        

        self.Waypoint = Pose()
        self.Waypoints = PoseArray()
        self.Waypoints_reached = PoseArray()

        self.tf = TransformListener()

        self.Target = []
        # Tolerances
        self.backoff = True
        self.distance_lower = 0.4 # Lower limit, if waypoint is closer, move back
        self.distance_keep  = 0.8 # Goal, keep this distance to the target
        self.distance_upper = 1.2 # Upper limit, if waypoint is further away, move closer

        self.distance_threshold = 0.1 # Distance to a waypoint before it is discarded

        self.break_threshold = 0.2

        self.twist_threshold = 30*math.pi/180

        # Subscribed topic
        self.pub_goal = rospy.Publisher("/Tracker/move_base_goal", PoseStamped, queue_size=1)
        self.pub_waypoint_list = rospy.Publisher("/Tracker/Visualization/Waypoint_list",PoseArray,queue_size=1)
        self.pub_waypoint_list_reached = rospy.Publisher("/Tracker/Visualization/Waypoint_list_deleted",PoseArray,queue_size=1)
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
        
        self.Move_base_goal     = self.Current_position.pose
        self.Waypoint_goal_pose = self.Current_position.pose

        rospy.Subscriber("/odometry/filtered_map",Odometry,self.Compare_pose,queue_size=1)
        rospy.Subscriber("/Tracker/Object_Tracker/Published_pose",PoseStamped,self.New_input, queue_size=1)
        #rospy.Subscriber("/Tracker/move_base_goal",PoseStamped,self.Waypoint_goal,queue_size=1)
        rospy.Subscriber("/move_base/Current_goal",PoseStamped,self.Current_goal, queue_size=1)
       

    def New_input(self,Pose):
        if rospy.has_param('/Target_UID'):
            if self.Target != rospy.get_param('/Target_UID'):
                self.Waypoints = PoseArray()
            self.Target = rospy.get_param('/Target_UID')
        
        #Pose.header.frame_id = "map"
        # Calculate Transforms and the Poses in their respective frames
        # Transform from "Pose Frame" to map
        transform_pm = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        # Pose in map frame
        np_m = tf2_geometry_msgs.do_transform_pose(Pose,transform_pm)
        # Mapping only takes zero values in z
        np_m.pose.position.z = 0 

        # Generate new waypoint
        self.Waypoints.header = Pose.header
        Waypoint = Pose.pose
        # If there are no current waypoint 
        distance_to_vehicle = math.sqrt((np_m.pose.position.x-self.Current_position.pose.pose.position.x)**2+(np_m.pose.position.y-self.Current_position.pose.pose.position.y)**2)
        
        if len(self.Waypoints.poses) == 0:
            if distance_to_vehicle < self.distance_keep:
                # If first target is within keep distance, do nothing.
                #print("No current waypoint, target is still too close")
                pass
            else:
                # If first target is further away than keep distance add it as a waypoint
                #print("No current waypoint, one is added")
                vec_new_old=np.array([np_m.pose.position.x,np_m.pose.position.y,np_m.pose.position.z])
                quaternion=get_new_orientation(self.Move_base_goal,np_m,0,Points=True)
                Waypoint.position.x = self.Current_position.pose.pose.position.x
                Waypoint.position.y = self.Current_position.pose.pose.position.y
                Waypoint.position.z = self.Current_position.pose.pose.position.z
                Waypoint.orientation.x = quaternion[0]
                Waypoint.orientation.y = quaternion[1]
                Waypoint.orientation.z = quaternion[2]
                Waypoint.orientation.w = quaternion[3]
                self.Waypoints.poses.append(Waypoint)
                self.pub_waypoint_list.publish(self.Waypoints)

        else:
            #print("{} waypoints".format(len(self.Waypoints.poses)))
            distance_to_goal = math.sqrt((np_m.pose.position.x-self.Waypoints.poses[-1].position.x)**2+(np_m.pose.position.y-self.Waypoints.poses[-1].position.y)**2)
            if distance_to_goal == 0:
                pass
            elif distance_to_goal < self.distance_lower and self.backoff == True:
                # If closer to object than lower limit, move back waypoint
                if len(self.Waypoints.poses) == 1:
                    lp_m = PoseStamped()
                    lp_m.pose = self.Waypoints.poses[-1]
                    np_m_vec=np.array([np_m.pose.position.x,np_m.pose.position.y,np_m.pose.position.z])
                    quaternion=get_new_orientation(self.Current_position.pose,np_m,0,Points=True)

                    Waypoint_coord = np.array([self.Waypoints.poses[-1].position.x,self.Waypoints.poses[-1].position.y])
                    Vehicle_coord  = np.array([self.Current_position.pose.pose.position.x,self.Current_position.pose.pose.position.y])
                    Object_coord   = np.array([np_m.pose.position.x,np_m.pose.position.y])
                    move_back_coord = cal_pose_stop(Object_coord,Vehicle_coord,self.distance_keep)

                    self.Waypoints.poses[-1].position.x = move_back_coord[0]
                    self.Waypoints.poses[-1].position.y = move_back_coord[1]

                    self.Waypoints.poses[-1].orientation.x = quaternion[0]
                    self.Waypoints.poses[-1].orientation.y = quaternion[1]
                    self.Waypoints.poses[-1].orientation.z = quaternion[2]
                    self.Waypoints.poses[-1].orientation.w = quaternion[3]
                    self.pub_waypoint_list.publish(self.Waypoints)

                    Waypoint = PoseStamped()
                    Waypoint.header = Pose.header
                    Waypoint.pose = self.Waypoints.poses[0]
                    self.pub_goal.publish(Waypoint)
                    self.Move_base_goal = Waypoint
                    #print(quaternion,"lower 1")
                else:
                    lp_m = PoseStamped()
                    lp_m.pose = self.Waypoints.poses[-1]
                    quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                    self.Waypoints.poses[-1].orientation.x = quaternion[0]
                    self.Waypoints.poses[-1].orientation.y = quaternion[1]
                    self.Waypoints.poses[-1].orientation.z = quaternion[2]
                    self.Waypoints.poses[-1].orientation.w = quaternion[3]
                    
                    self.pub_waypoint_list.publish(self.Waypoints)
                    #print(quaternion,"lower 2")

            elif distance_to_goal < self.distance_upper:
                # If object is between upper and lower limit change angle of waypoint
                lp_m = PoseStamped()
                lp_m.pose = self.Waypoints.poses[-1]
                quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                self.Waypoints.poses[-1].orientation.x = quaternion[0]
                self.Waypoints.poses[-1].orientation.y = quaternion[1]
                self.Waypoints.poses[-1].orientation.z = quaternion[2]
                self.Waypoints.poses[-1].orientation.w = quaternion[3]
                self.pub_waypoint_list.publish(self.Waypoints)
                #print(quaternion,"upper 1")
            else:
                # If object is further away, evaluate distance to current goal
                distance_to_goal = math.sqrt((np_m.pose.position.x-self.Waypoints.poses[-1].position.x)**2+(np_m.pose.position.y-self.Waypoints.poses[-1].position.y)**2)
                if self.distance_keep < distance_to_goal:
                    lp_m = PoseStamped()
                    lp_m.pose = self.Waypoints.poses[-1]
                    quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                    Waypoint.position.x = np_m.pose.position.x
                    Waypoint.position.y = np_m.pose.position.y
                    Waypoint.position.z = np_m.pose.position.z
                    Waypoint.orientation.x = quaternion[0]
                    Waypoint.orientation.y = quaternion[1]
                    Waypoint.orientation.z = quaternion[2]
                    Waypoint.orientation.w = quaternion[3]
                    self.Waypoints.poses.append(Waypoint)
                    self.pub_waypoint_list.publish(self.Waypoints)
                    #print(quaternion,"else 1")
                else:
                    lp_m = PoseStamped()
                    lp_m.pose = self.Waypoints.poses[-1]
                    quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                    self.Waypoints.poses[-1].orientation.x = quaternion[0]
                    self.Waypoints.poses[-1].orientation.y = quaternion[1]
                    self.Waypoints.poses[-1].orientation.z = quaternion[2]
                    self.Waypoints.poses[-1].orientation.w = quaternion[3]
                    self.pub_waypoint_list.publish(self.Waypoints)
                    #print(quaternion,"else 2")

    def Compare_pose(self,Pose):
        self.Current_position = Pose
        Waypoint = PoseStamped()
        Waypoint.header = Pose.header

        if rospy.has_param('/Target_UID'):
            if self.Target != rospy.get_param('/Target_UID'):
                if self.Move_base_goal != []:

                    #distance_between_vehicle_and_goal = math.sqrt((self.Move_base_goal.pose.position.x-self.Current_position.pose.pose.position.x)**2+(self.Move_base_goal.pose.position.y-self.Current_position.pose.pose.position.y)**2)
                    distance_between_vehicle_and_goal = math.sqrt((self.Waypoint_goal_pose.pose.position.x-self.Current_position.pose.pose.position.x)**2+(self.Waypoint_goal_pose.pose.position.y-self.Current_position.pose.pose.position.y)**2)
                    
                    if distance_between_vehicle_and_goal > self.break_threshold:
                        Goal_coord      = np.array([self.Waypoint_goal_pose.pose.position.x,self.Waypoint_goal_pose.pose.position.y])
                        #Goal_coord      = np.array([self.Move_base_goal.pose.position.x,self.Move_base_goal.pose.position.y])
                        Vehicle_coord   = np.array([self.Current_position.pose.pose.position.x,self.Current_position.pose.pose.position.y])

                        #quaternion=get_new_orientation(self.Current_position.pose,self.Move_base_goal,0,Points=True)
                        quaternion=get_new_orientation(self.Current_position.pose,self.Waypoint_goal_pose,0,Points=True)

                        move_back_coord = cal_pose_stop(Vehicle_coord,Goal_coord,self.break_threshold)
                        Waypoint.pose.position.x = move_back_coord[0]
                        Waypoint.pose.position.y = move_back_coord[1]

                        Waypoint.pose.orientation.x = quaternion[0]
                        Waypoint.pose.orientation.y = quaternion[1]
                        Waypoint.pose.orientation.z = quaternion[2]
                        Waypoint.pose.orientation.w = quaternion[3]
                        self.pub_goal.publish(Waypoint)
                        self.Move_base_goal = Waypoint
                
                self.Waypoints = PoseArray()
                self.Waypoints.header = Pose.header
                self.pub_waypoint_list.publish(self.Waypoints)
            self.Target = rospy.get_param('/Target_UID')

        
        Waypoint_reached = PoseStamped()
        Waypoint_reached.header = Pose.header
        self.Waypoints_reached.header = Pose.header

        # Check if there is a current goal
        if self.Move_base_goal != []:
            #distance_to_vehicle = math.sqrt((np_m.pose.position.x-self.Current_position.pose.pose.position.x)**2+(np_m.pose.position.y-self.Current_position.pose.pose.position.y)**2)
            #distance_to_goal = math.sqrt((self.Move_base_goal.pose.position.x-self.Current_position.pose.pose.position.x)**2+(self.Move_base_goal.pose.position.y-self.Current_position.pose.pose.position.y)**2)
            distance_to_goal = math.sqrt((self.Waypoint_goal_pose.pose.position.x-self.Current_position.pose.pose.position.x)**2+(self.Waypoint_goal_pose.pose.position.y-self.Current_position.pose.pose.position.y)**2)
            #print(distance_to_goal, "d2g")
            if len(self.Waypoints.poses) == 0:
                # If len of waypoints equal 0 do nothing, this is only the case when no object have been found.
                pass
            elif len(self.Waypoints.poses) == 1:
                
                if distance_to_goal < self.distance_threshold:
                    #Q_goal = Quaternion(self.Move_base_goal.pose.orientation.w,self.Move_base_goal.pose.orientation.x,self.Move_base_goal.pose.orientation.y,self.Move_base_goal.pose.orientation.z)
                    Q_goal = Quaternion(self.Waypoint_goal_pose.pose.orientation.w,self.Waypoint_goal_pose.pose.orientation.x,self.Waypoint_goal_pose.pose.orientation.y,self.Waypoint_goal_pose.pose.orientation.z)
                    Q_wp   = Quaternion(self.Waypoints.poses[0].orientation.w,self.Waypoints.poses[0].orientation.x,self.Waypoints.poses[0].orientation.y,self.Waypoints.poses[0].orientation.z)
                    Twist_goal = abs((Q_goal*Q_wp.inverse).angle)
                    if Twist_goal > self.twist_threshold:
                        Waypoint.pose = self.Waypoints.poses[0]
                        Waypoint_reached.pose = self.Waypoint_goal_pose.pose
                        self.Waypoints_reached.poses.append(Waypoint_reached.pose)
                        self.pub_goal.publish(Waypoint)
                        self.Move_base_goal = Waypoint
                        self.pub_waypoint_list_reached.publish(self.Waypoints_reached)

                distance_between_goal_and_waypoint = math.sqrt((self.Move_base_goal.pose.position.x-self.Waypoints.poses[0].position.x)**2+(self.Move_base_goal.pose.position.y-self.Waypoints.poses[0].position.y)**2)
                #distance_between_goal_and_waypoint = math.sqrt((self.Waypoint_goal_pose.pose.position.x-self.Waypoints.poses[0].position.x)**2+(self.Waypoint_goal_pose.pose.position.y-self.Waypoints.poses[0].position.y)**2)
                if distance_between_goal_and_waypoint != 0:
                    Waypoint.pose = self.Waypoints.poses[0]
                    self.pub_goal.publish(Waypoint)
                    self.Move_base_goal = Waypoint
            elif len(self.Waypoints.poses) == 2:
                if distance_to_goal < self.distance_threshold:
                    self.Waypoints_reached.poses.append(self.Waypoints.poses[0])
                    del self.Waypoints.poses[0]

                    Waypoint.pose = self.Waypoints.poses[0]
                    #quaternion=get_new_orientation(self.Current_position.pose,Waypoint,0,Points=True)
                    quaternion=get_new_orientation(self.Move_base_goal,Waypoint,0,Points=True)

                    # Måske skal det skiftes til currnet object position i stedet for Waypoint
                    Waypoint_coord = np.array([self.Waypoints.poses[0].position.x,self.Waypoints.poses[0].position.y]) ## Ændret
                    Move_base_coord = np.array([self.Move_base_goal.pose.position.x,self.Move_base_goal.pose.position.y])
                    #Vehicle_coord  = np.array([self.Current_position.pose.pose.position.x,self.Current_position.pose.pose.position.y])

                    move_back_coord = cal_pose_stop(Waypoint_coord,Move_base_coord,self.distance_keep)
                    #move_back_coord = cal_pose_stop(Waypoint_coord,Vehicle_coord,self.distance_keep)


                    self.Waypoints.poses[0].position.x = move_back_coord[0]
                    self.Waypoints.poses[0].position.y = move_back_coord[1]

                    self.Waypoints.poses[0].orientation.x = quaternion[0]
                    self.Waypoints.poses[0].orientation.y = quaternion[1]
                    self.Waypoints.poses[0].orientation.z = quaternion[2]
                    self.Waypoints.poses[0].orientation.w = quaternion[3]
                    Waypoint.pose = self.Waypoints.poses[0]

                    self.pub_waypoint_list_reached.publish(self.Waypoints_reached)
                    self.pub_waypoint_list.publish(self.Waypoints)
                    self.pub_goal.publish(Waypoint)
                    self.Move_base_goal = Waypoint
            else:
                if distance_to_goal < self.distance_threshold:
                    self.Waypoints_reached.poses.append(self.Waypoints.poses[0])
                    del self.Waypoints.poses[0]
                    Waypoint.pose = self.Waypoints.poses[0]
                    self.pub_goal.publish(Waypoint)
                    self.Move_base_goal = Waypoint
                    self.pub_waypoint_list_reached.publish(self.Waypoints_reached)
                    self.pub_waypoint_list.publish(self.Waypoints)
        else:
            if len(self.Waypoints.poses) != 0:
                Waypoint.pose = self.Waypoints.poses[0]
                self.pub_goal.publish(Waypoint)
                self.Move_base_goal = Waypoint

    def Waypoint_goal(self,Pose):
        self.Move_base_goal = Pose

    def Current_goal(self,Pose):
        self.Waypoint_goal_pose = Pose

if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass