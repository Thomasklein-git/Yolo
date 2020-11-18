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
        self.distance_keep = 0.4 #[m]
        self.distance_new = 0.8
        self.distance_threshold = 0.3 # Distance to a waypoint before it is discarded
        self.twist_threshold = 30*math.pi/180

        # Subscribed topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_waypoint_list = rospy.Publisher("/Tracker/Waypoint_list",PoseArray,queue_size=1)
        self.pub_waypoint_list_reached = rospy.Publisher("/Tracker/Waypoint_list_deleted",PoseArray,queue_size=1)
        print("Waiting for Odometry...")
        try:
            self.Current_position = rospy.wait_for_message("/odometry/filtered_map",Odometry,timeout=5)
        except:
            self.Current_position = Odometry()
            self.Current_position.pose.pose.position.x = 0
            self.Current_position.pose.pose.position.y = 0
            self.Current_position.pose.pose.position.z = 0
            self.Current_position.pose.pose.orientation.w = 1
            self.Current_position.pose.pose.orientation.x = 0
            self.Current_position.pose.pose.orientation.y = 0
            self.Current_position.pose.pose.orientation.z = 0
        
        self.Move_base_goal = self.Current_position.pose
        
        print("Odometry received")
        rospy.Subscriber("/odometry/filtered_map",Odometry,self.Compare_pose,queue_size=1)
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

        # If self.Waypoints is empty 
        if self.Waypoints.poses == []:
            # Check distance to the current point from vehicle
            distance_to_vehicle = math.sqrt((np_m.pose.position.x-self.Current_position.pose.pose.position.x)**2+(np_m.pose.position.y-self.Current_position.pose.pose.position.y)**2)
            # If point is within proximity of vehicle, add current vehicle position with orientation towards point
            if distance_to_vehicle < self.distance_new:
                vec_new_old=np.array([np_m.pose.position.x,np_m.pose.position.y,np_m.pose.position.z])
                quaternion=get_new_orientation(self.Move_base_goal,np_m,0,Points=True)

                Waypoint.position.x = self.Move_base_goal.pose.position.x
                Waypoint.position.y = self.Move_base_goal.pose.position.y
                Waypoint.position.z = self.Move_base_goal.pose.position.z
            # If point is NOT witin proximity of vehicle, add object position with orientation towards point
            else:
                print("case 2")
                vec_new_old=np.array([np_m.pose.position.x,np_m.pose.position.y,np_m.pose.position.z])
                quaternion=get_new_orientation(0,0,vec_new_old,Points=False)

                Waypoint.position.x = np_m.pose.position.x
                Waypoint.position.y = np_m.pose.position.y
                Waypoint.position.z = np_m.pose.position.z

            Waypoint.orientation.x = quaternion[0]
            Waypoint.orientation.y = quaternion[1]
            Waypoint.orientation.z = quaternion[2]
            Waypoint.orientation.w = quaternion[3]

            self.Waypoints.poses.append(Waypoint)
            self.pub_waypoint_list.publish(self.Waypoints)
        
        # If self.Waypoints is NOT empty
        else:
            # If within proximity of the latest waypoint update the orientation of the waypoint
            lp_m = PoseStamped()
            lp_m.pose = self.Waypoints.poses[-1]
            distance_to_lastest_waypoint = math.sqrt((np_m.pose.position.x-lp_m.pose.position.x)**2+(np_m.pose.position.y-lp_m.pose.position.y)**2)
            if distance_to_lastest_waypoint < self.distance_new:
                print("case 3")
                quaternion=get_new_orientation(lp_m,np_m,0,Points=True)

                self.Waypoints.poses[-1].orientation.x = quaternion[0]
                self.Waypoints.poses[-1].orientation.y = quaternion[1]
                self.Waypoints.poses[-1].orientation.z = quaternion[2]
                self.Waypoints.poses[-1].orientation.w = quaternion[3]    

                self.pub_waypoint_list.publish(self.Waypoints)        

            # If NOT within proximity of the latast waypoint add as a new point
            else:
                print("case 4")
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
        
        #print(self.Waypoints)

        """
        transform = self.tf_buffer.lookup_transform("base_link", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        np_b = tf2_geometry_msgs.do_transform_pose(Pose, transform) # New Waypoint in base
        npd = math.sqrt((np_b.pose.position.x)**2+(np_b.pose.position.y)**2)
        if npd > self.distance_threshold:

            transform_np = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            np_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_np)
            np_m.pose.position.z = 0

            # If the waypoint is further away than distance_threshold and Waypoint list is empty add point to waypoints
            # If list is not empty compare new waypoint with the latest added waypoint to ensure that distance between the two points are greater than distance_new
            if self.Waypoints2.poses == []:
                vec_new_old=np.array([np_m.pose.position.x,np_m.pose.position.y,np_m.pose.position.z])
                quaternion=get_new_orientation(0,0,vec_new_old,Points=False)
                np_m.pose.orientation.x=quaternion[0]
                np_m.pose.orientation.y=quaternion[1]
                np_m.pose.orientation.z=quaternion[2]
                np_m.pose.orientation.w=quaternion[3]
               
                self.update_waypoints(np_m)
                
            else: 
                lp_m = PoseStamped()
                lp_m.pose = self.Waypoints2.poses[-1]
                quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                np_m.pose.orientation.x=quaternion[0]
                np_m.pose.orientation.y=quaternion[1]
                np_m.pose.orientation.z=quaternion[2]
                np_m.pose.orientation.w=quaternion[3]
                wpd = math.sqrt((np_m.pose.position.x-lp_m.pose.position.x)**2+(np_m.pose.position.y-lp_m.pose.position.y)**2)
                if wpd > self.distance_new:
                    #self.Waypoints.append(np_m)
                    self.update_waypoints(np_m)

        
        # If point is closer than distance treshold and goal is reached (Only on point in the list), initiate rotation on the spot
        else:
            pass
        """
    
    def Compare_pose(self,Pose):
        # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
        #print(len(self.Waypoints2.poses),"Compare")
        self.Current_position = Pose

        Waypoint = PoseStamped()
        Waypoint.header = Pose.header
        self.Waypoints_reached.header = Pose.header

        #self.Waypoints

        # Check if there is a current goal
        if self.Move_base_goal != []:
            # Check the distance from the vehicle to the current goal
            distance_to_goal = math.sqrt((self.Current_position.pose.pose.position.x-self.Move_base_goal.pose.position.x)**2+(self.Current_position.pose.pose.position.y-self.Move_base_goal.pose.position.y)**2)
            # If vehicle is within distance_threshold:
            
            if distance_to_goal < self.distance_threshold:
                # Check lenght of Waypoint.poses
                # If empty do nothing
                if len(self.Waypoints.poses) == 0:
                    pass
                # If list is 1 waypoint 
                elif len(self.Waypoints.poses) == 1:
                    # If angle between goal and current_waypoint is greater than twist_threshold
                    Q_goal = Quaternion(self.Move_base_goal.pose.orientation.w,self.Move_base_goal.pose.orientation.x,self.Move_base_goal.pose.orientation.y,self.Move_base_goal.pose.orientation.z)
                    Q_wp   = Quaternion(self.Waypoints.poses[0].orientation.w,self.Waypoints.poses[0].orientation.x,self.Waypoints.poses[0].orientation.y,self.Waypoints.poses[0].orientation.z)
                    Twist_goal = abs((Q_goal*Q_wp.inverse).angle)
                    if Twist_goal > self.twist_threshold: 
                        Waypoint.pose = self.Waypoints.poses[0]
                        self.pub_goal.publish(Waypoint)
                        self.Waypoints_reached.poses.append(self.Waypoints.poses[0])
                        del self.Waypoints.poses[0]
                        self.pub_waypoint_list_reached.publish(self.Waypoints_reached)
                        self.pub_waypoint_list.publish(self.Waypoints)
                    pass
                    # c
                # If list have more than one waypoint
                else:
                    self.Waypoints_reached.poses.append(self.Waypoints.poses[0])
                    del self.Waypoints.poses[0]
                    self.pub_waypoint_list_reached.publish(self.Waypoints_reached)
                    Waypoint.pose = self.Waypoints.poses[0]
                    self.pub_goal.publish(Waypoint)
                    self.pub_waypoint_list.publish(self.Waypoints)


            # distance to the goal is further than distance_threshold
            else:
                pass

        # If there is no current goal publish the current waypoint if there is any
        else:
            if len(self.Waypoints.poses) > 0:
                Waypoint.pose = self.Waypoints.poses[0]
                self.pub_goal.publish(Waypoint)
        """


        # If vehicle is not within distance_threshold, keep pursuing goal
        else:
            pass


        #Ch_vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
        if len(Waypoints.poses) == 0:
            pass
        else:
            

            # Distance between to goal.
            if self.Move_base_goal != []
                distance_to_goal = math.sqrt((np_m.pose.position.x-self.Move_base_goal.pose.position.x)**2+(np_m.pose.position.y-self.Move_base_goal.pose.position.y)**2)
            # If there is no goal, publish the Waypoint as a goal
            else:    
                Waypoint.pose = Waypoints.poses[0]
                self.pub_goal.publish(Waypoint)
        """

    

        """
        # Check if distance to vehicle is less that distance keep
            #distance_to_vehicle = math.sqrt((np_m.pose.position.x-self.Current_position.pose.pose.position.x)**2+(np_m.pose.position.y-self.Current_position.pose.pose.position.y)**2)
            
            # If there is a goal calculate the distance between the current only waypoint and the goal
            if self.Move_base_goal != []
                distance_to_goal    = math.sqrt((np_m.pose.position.x-self.Move_base_goal.pose.position.x)**2+(np_m.pose.position.y-self.Move_base_goal.pose.position.y)**2)
                # If the distance between the current goal and current waypoint is less that self.distance_new, this point is a current distance point and the angle needs to be updated
                if distance_to_goal < self.distance_new:

                    pass
                # If the distance_to_goal is larger than current goal 
                else:
                    pass

            # If the distance to the point is within distance_keep
            if distance_to_vehicle < self.distance_new:
                pass
            else:
                pass
                # If the angle between the current goal and the 
                # 
        """
        """
        # Responsible of publishing the goal for the vehicle to move to.
        # If waypoint list is empty do nothing
        if len(Waypoints.poses) == 0:
            pass
        # If waypoint list only contains one pose
        elif len(Waypoints.poses) == 1:
            Waypoint = Waypoints.poses[0]
            #print("CASE 2")

            #self.Waypoints.append(Waypoints[0])
            #self.pub_goal.publish(Waypoints[0])
            Goal_m=Waypoints.poses[0]
            #print(self.Waypoints[0].pose.position,"i 2")

            #print(Goal_m.pose.position,"--->goal pose")
            vec_Goal_map = np.array([Goal_m.position.x,Goal_m.position.y])
            #print(self.Waypoints[0].pose.position,"i 3")

            #print(vec_Goal_map,"Goal")
            vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
            #print(self.Waypoints[0].pose.position,"i 4")

            #print(vec_Vehicle_map,"Vehicle")
            xy_goal_stop = cal_pose_stop(vec_Goal_map,vec_Vehicle_map,self.distance_keep)

            #temp_waypoint=Goal_m
            temp_waypoint = self.Waypoints2.poses[0]
            #Goal_m.pose.position.x=xy_goal_stop[0] # Udkommenter hvis den skal køre ind i object
            #Goal_m.pose.position.y=xy_goal_stop[1] # Udkommenter hvis den skal køre ind i object

            temp_pose = PoseStamped()
            temp_pose.header.stamp = rospy.Time.now()
            temp_pose.header.frame_id = "map"
            temp_pose.pose.position.x = xy_goal_stop[0]
            temp_pose.pose.position.y = xy_goal_stop[1]
            temp_pose.pose.position.z = self.Waypoints2.poses[0].position.z
            temp_pose.pose.orientation.x = self.Waypoints2.poses[0].orientation.x
            temp_pose.pose.orientation.y = self.Waypoints2.poses[0].orientation.y
            temp_pose.pose.orientation.z = self.Waypoints2.poses[0].orientation.z
            temp_pose.pose.orientation.w = self.Waypoints2.poses[0].orientation.w
            #if temp_pose.pose.position.x - self.Waypoints[0].pose.position.x == 0 and temp_pose.pose.position.y - self.Waypoints[0].pose.position.y == 0:
            self.pub_goal.publish(temp_pose)

        # If waypoint list contains poses
            
        else:
            #print("CASE 3")
            # Calculate the distance to all current waiting waypoints
            d2pb = []
            transform_bm = self.tf_buffer.lookup_transform("map","base_link", Pose.header.stamp, rospy.Duration(1.0))
            transform_mb = self.tf_buffer.lookup_transform("base_link","map", Pose.header.stamp, rospy.Duration(1.0))
            for Point_m in Waypoints.poses:
                Point = PoseStamped() 
                Point.pose = Point_m
                Point_b = tf2_geometry_msgs.do_transform_pose(Point, transform_mb)
                d2pb.append(math.sqrt(Point_b.pose.position.x**2+Point_b.pose.position.y**2))
            minpos = d2pb.index(min(d2pb))
            # If any waypoints in the list is closer to the base, discard all waypoints up to the closest point.
            if minpos > 0:
                for i in range(0,minpos):
                    del Waypoints.poses[0]

            
            # Check if the current Waypoint in 
            Goal_m    = Waypoints.poses[0]
            goal = PoseStamped()
            goal.pose = Goal_m # Need fix
            Goal_b    = tf2_geometry_msgs.do_transform_pose(goal, transform_mb)
            distance2waypoint = math.sqrt(Goal_b.pose.position.x**2+Goal_b.pose.position.y**2)
            #print(distance2waypoint,"distance")
            # If the waypoint is further away than distance threshold, check if point is the current goal or else publish as current goal
            if distance2waypoint > self.distance_threshold:
                # If current waypoint is the same as goal, pass
                if self.Move_base_goal == Waypoints.poses[0]:
                    pass
                # If current waypoint is not same as goal, publish current waypoint and add current goal
                else:
                    pubpoint = PoseStamped()
                    pubpoint.header = Waypoints.header
                    pubpoint.header.stamp = rospy.Time.now()
                    pubpoint.pose = Waypoints.poses[0]
                    """
        """
                    pubpoint.pose.orientation.x = 0
                    pubpoint.pose.orientation.y = 0
                    pubpoint.pose.orientation.z = 0
                    pubpoint.pose.orientation.w = 1
                    """
        """
                    
                    self.pub_goal.publish(pubpoint)
            # If the waypoint is within distance threshold remove current waypoint 
            else:
                if len(Waypoints.poses) != 1:
                    del Waypoints.poses[0]
                elif len(Waypoints.poses)==0:
                    pass
                else:
                    pubpoint = PoseStamped()
                    pubpoint.header = Waypoints.header
                    pubpoint.header.stamp = rospy.Time.now()
                    pubpoint.pose = Waypoints.poses[0]
                    """
        """
                    pubpoint.pose.orientation.x = 0
                    pubpoint.pose.orientation.y = 0
                    pubpoint.pose.orientation.z = 0
                    pubpoint.pose.orientation.w = 1
                    """
        """

                    self.pub_goal.publish(pubpoint)
                """
        """
                del Waypoints[0]
                # If list is now empty do nothing
                if len(Waypoints) == 0:
                    pass
                # If list contains one pose do not publish 
                elif len(Waypoints) == 1:
                    pass
                # If list containt more poses, publish a new goal
                else:
                    self.pub_goal.publish(Waypoints[0])
                """
                

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