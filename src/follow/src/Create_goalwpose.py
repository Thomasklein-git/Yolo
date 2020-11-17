#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
from tf import TransformListener
import math
from agfh import *

class Follow():
    def __init__(self):
        rospy.init_node('Create_goal')

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.Waypoints = []
        self.Move_base_goal = []
        self.Waypoints2 = PoseArray()

        self.tf = TransformListener()
        # Tolerances
        self.distance_keep = 0.5 #[m]
        self.distance_new = 0.6
        self.distance_threshold = 0.3 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_waypoint_list = rospy.Publisher("/Tracker/Waypoint_list",PoseArray,queue_size=1)

        rospy.Subscriber("/odometry/filtered_map",Odometry,self.Compare_pose,queue_size=1)
        rospy.Subscriber("/Tracker/Object_Tracker/Published_pose",PoseStamped,self.New_input, queue_size=1)
        rospy.Subscriber("/move_base/Current_goal",PoseStamped,self.Current_goal, queue_size=1)
        
        
    def New_input(self,Pose):
        #print(Pose, "Pose")
        #print(self.Waypoints2)
       # Waypoints = self.Waypoints
        transform = self.tf_buffer.lookup_transform("base_link", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        
        np_b = tf2_geometry_msgs.do_transform_pose(Pose, transform) # New Waypoint in base
        print(np_b, "np_b")
        print(Pose,"Pose")
        npd = math.sqrt((np_b.pose.position.x)**2+(np_b.pose.position.y)**2)

        # Check if point from Zed cam is further away than distance_threshold
        if npd > self.distance_threshold:
            transform_np = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            np_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_np)
            np_m.pose.position.z = 0

            # If the waypoint is further away than distance_threshold and Waypoint list is empty add point to waypoints
            # If list is not empty compare new waypoint with the latest added waypoint to ensure that distance between the two points are greater than distance_new
            if self.Waypoints2.poses == []:
                vec_new_old=np.array([np_b.pose.position.x,np_b.pose.position.y,np_b.pose.position.z])
                quaternion=get_new_orientation(0,0,vec_new_old,Points=False)
                np_m.pose.orientation.x=quaternion[0]
                np_m.pose.orientation.y=quaternion[1]
                np_m.pose.orientation.z=quaternion[2]
                np_m.pose.orientation.w=quaternion[3]
                #print(np_m,"empty")
                self.update_waypoints(np_m)
                
            else: 
                lp_m = PoseStamped()
                lp_m.pose = self.Waypoints2.poses[-1]
                quaternion=get_new_orientation(lp_m,np_m,0,Points=True)
                np_m.pose.orientation.x=quaternion[0]
                np_m.pose.orientation.y=quaternion[1]
                np_m.pose.orientation.z=quaternion[2]
                np_m.pose.orientation.w=quaternion[3]
                #print(np_m,"not empty")
                wpd = math.sqrt((np_m.pose.position.x-lp_m.pose.position.x)**2+(np_m.pose.position.y-lp_m.pose.position.y)**2)
                if wpd > self.distance_new:
                    #self.Waypoints.append(np_m)
                    self.update_waypoints(np_m)
    
    def Compare_pose(self,Pose):
        # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
        print(len(self.Waypoints2.poses),"Compare")
        Waypoints=self.Waypoints2
        Ch_vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
        #print(Ch_vec_Vehicle_map,"Vehicle before")
        #print(Waypoints[0].pose.position.x,Waypoints[0].pose.position.y,"--->last point")
        #print(Waypoints[-1].pose.position.x,Waypoints[-1].pose.position.y,"--->new point")
        #print(len(Waypoints.poses),"length")
        # If waypoint list is empty do nothing
        if len(Waypoints.poses) == 0:
            #print("CASE 1")
            pass
        # If waypoint list only contains one pose
            
        elif len(Waypoints.poses) == 1:
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
            temp_pose.pose.orientation.x = 0#self.Waypoints2.poses[0].orientation.x
            temp_pose.pose.orientation.y = 0#self.Waypoints2.poses[0].orientation.y
            temp_pose.pose.orientation.z = -0.866 #self.Waypoints2.poses[0].orientation.z
            temp_pose.pose.orientation.w = 0.5#self.Waypoints2.poses[0].orientation.w

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
                print("hej")
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
                    pubpoint.pose.orientation.x = 0
                    pubpoint.pose.orientation.y = 0
                    pubpoint.pose.orientation.z = -0.866
                    pubpoint.pose.orientation.w = 0.5
                    
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
                    pubpoint.pose.orientation.x = 0
                    pubpoint.pose.orientation.y = 0
                    pubpoint.pose.orientation.z = -0.866
                    pubpoint.pose.orientation.w = 0.5

                    self.pub_goal.publish(pubpoint)
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