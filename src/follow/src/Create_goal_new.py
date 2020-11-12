#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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
        self.distance_keep = 0.3 #[m]
        self.distance_new = 1
        self.distance_threshold = 0.5 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        rospy.Subscriber("/odometry/filtered_map",Odometry,self.Compare_pose,queue_size=1)
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
        Waypoints=self.Waypoints
        Ch_vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
        print(Ch_vec_Vehicle_map,"Vehicle before")
        print(Waypoints[0].pose.position.x,Waypoints[0].pose.position.y,"--->last point")
        print(Waypoints[-1].pose.position.x,Waypoints[-1].pose.position.y,"--->new point")
        print(len(Waypoints),"length")
        # If waypoint list is empty do nothing
        if len(Waypoints) == 0:
            print("CASE 1")
            pass
        # If waypoint list only contains one pose
        elif len(Waypoints) == 1:
            print("CASE 2")
            #self.Waypoints.append(Waypoints[0])
            #self.pub_goal.publish(Waypoints[0])
            Goal_m=Waypoints[0]
            #print(Goal_m.pose.position,"--->goal pose")
            vec_Goal_map = np.array([Goal_m.pose.position.x,Goal_m.pose.position.y])
            print(vec_Goal_map,"Goal")
            vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
            print(vec_Vehicle_map,"Vehicle")
            xy_goal_stop = cal_pose_stop(vec_Goal_map,vec_Vehicle_map,self.distance_keep)
            temp_waypoint=Goal_m
            temp_waypoint.pose.position.x=xy_goal_stop[0] # Udkommenter hvis den skal køre ind i object
            temp_waypoint.pose.position.y=xy_goal_stop[1] # Udkommenter hvis den skal køre ind i object

            self.pub_goal.publish(temp_waypoint)

        # If waypoint list contains poses
        else:
            print("CASE 3")
            # Calculate the distance to all current waiting waypoints
            d2pb = []
            transform_bm = self.tf_buffer.lookup_transform("map","base_link", Pose.header.stamp, rospy.Duration(1.0))
            transform_mb = self.tf_buffer.lookup_transform("base_link","map", Pose.header.stamp, rospy.Duration(1.0))
            for Point_m in Waypoints:
                Point_b = tf2_geometry_msgs.do_transform_pose(Point_m, transform_mb)
                d2pb.append(math.sqrt(Point_b.pose.position.x**2+Point_b.pose.position.y**2))
            minpos = d2pb.index(min(d2pb))
            # If any waypoints in the list is closer to the base, discard all waypoints up to the closest point.
            if minpos > 0:
                print("hej")
                for i in range(0,minpos):
                    del Waypoints[0]
            
            # Check if the current Waypoint in 
            Goal_m    = Waypoints[0]
            Goal_b    = tf2_geometry_msgs.do_transform_pose(Goal_m, transform_mb)
            distance2waypoint = math.sqrt(Goal_b.pose.position.x**2+Goal_b.pose.position.y**2)
            print(distance2waypoint,"distance")
            # If the waypoint is further away than distance threshold, check if point is the current goal or else publish as current goal
            if distance2waypoint > self.distance_threshold:
                # If current waypoint is the same as goal, pass
                if self.Move_base_goal == Waypoints[0]:
                    pass
                # If current waypoint is not same as goal, publish current waypoint and add current goal
                else:
                    self.pub_goal.publish(Waypoints[0])
            # If the waypoint is within distance threshold remove current waypoint 
            else:
                if len(Waypoints) != 1:
                    del Waypoints[0]
                elif len(Waypoints)==0:
                    pass
                else:
                    self.pub_goal.publish(Waypoints[0])
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

    
    def Compare_pose_old(self,Pose):
        Waypoints=self.Waypoints
        # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
        # If waypoint list is empty do nothing
        if len(Waypoints) == 0:
            pass
        # If waypoint list contains points 
        else:
            print(Waypoints[0].pose.position,"CP")
            # Calculate the distance to all current waiting waypoints
            d2pb = []
            transform_bm = self.tf_buffer.lookup_transform("map","base_link", Pose.header.stamp, rospy.Duration(1.0))
            transform_mb = self.tf_buffer.lookup_transform("base_link","map", Pose.header.stamp, rospy.Duration(1.0))
            for Point_m in Waypoints:
                Point_b = tf2_geometry_msgs.do_transform_pose(Point_m, transform_mb)
                d2pb.append(math.sqrt(Point_b.pose.position.x**2+Point_b.pose.position.y**2))
            minpos = d2pb.index(min(d2pb))
            # If any waypoints in the list is closer to the base, discard all waypoints up to the closest point.
            if minpos > 0:
                for i in range(0,minpos):
                    del Waypoints[0]


            # Check if the current Waypoint in 
            Goal_m      = Waypoints[0]
            Goal_b    = tf2_geometry_msgs.do_transform_pose(Goal_m, transform_mb)
            distance2waypoint = math.sqrt(Goal_b.pose.position.x**2+Goal_b.pose.position.y**2)
            # if the waypoint is further away than distance threshold, check if point is the current goal or else publish as current goal
            if distance2waypoint > self.distance_threshold:
                # If current waypoint is the same as goal, pass
                if len(Waypoints) == 1:
                    print("1")
                    Goal_m=Waypoints[0]
                    vec_Goal_map = np.array([Goal_m.pose.position.x,Goal_m.pose.position.y])
                    vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
                    xy_goal_stop = cal_pose_stop(vec_Goal_map,vec_Vehicle_map,self.distance_keep)
                    temp_waypoint=Waypoints[0]
                    temp_waypoint.pose.position.x=xy_goal_stop[0]
                    temp_waypoint.pose.position.y=xy_goal_stop[1]
                    
                    self.pub_goal.publish(temp_waypoint)
                elif self.Move_base_goal == Waypoints[0]:
                    pass
                else:
                    self.pub_goal.publish(Waypoints[0])
                # If current waypoint is not same as goal, publish current waypoint and add current goal
                """
                else:
                    if len(self.Waypoints) == 1:
                        print("1")
                        Goal_m=self.Waypoints[0]
                        vec_Goal_map = np.array([Goal_m.pose.position.x,Goal_m.pose.position.y])
                        vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
                        xy_goal_stop = cal_pose_stop(vec_Goal_map,vec_Vehicle_map,self.distance_keep)
                        temp_waypoint=self.Waypoints[0]
                        temp_waypoint.pose.position.x=xy_goal_stop[0]
                        temp_waypoint.pose.position.y=xy_goal_stop[1]
                    
                        self.pub_goal.publish(temp_waypoint)
                    else:
                        self.pub_goal.publish(self.Waypoints[0])
                    #self.Move_base_goal = self.Waypoints[0]:
                """
            # If the waypoint is within distance threshold remove current waypoint 
                """
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
                """
            else:
                if len(Waypoints) != 1:
                    del self.Waypoints[0]
                # If list contains only one pose  
                if len(Waypoints) == 1:
                    print("2")
                    Goal_m=Waypoints[0]
                    vec_Goal_map = np.array([Goal_m.pose.position.x,Goal_m.pose.position.y])
                    print(vec_Goal_map,"Goal")
                    vec_Vehicle_map = np.array([Pose.pose.pose.position.x,Pose.pose.pose.position.y])
                    print(vec_Vehicle_map,"Vehicle")
                    xy_goal_stop = cal_pose_stop(vec_Goal_map,vec_Vehicle_map,self.distance_keep)
                    temp_waypoint=Goal_m
                    temp_waypoint.pose.position.x=xy_goal_stop[0]
                    temp_waypoint.pose.position.y=xy_goal_stop[1]
                    self.pub_goal.publish(temp_waypoint) 
                    
                    #del self.Waypoints[0]
                # If list containt more poses, publish a new goal
                else:
                    self.pub_goal.publish(Waypoints[0])
    
    def Current_goal(self,Pose):
        self.Move_base_goal = Pose

if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass