#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
from tf import TransformListener
import math

class Follow():
    def __init__(self):
        rospy.init_node('Poser', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.Waypoints = []

        self.tf = TransformListener()
        # Tolerances
        self.distance_keep = 0.5 #[m]
        self.distance_new = 1
        self.distance_threshold = 0.5 # Distance to a waypoint before it is discarded

        # Subscribed topic
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        rospy.Subscriber("/odometry/filtered_map",PoseStamped,self.Compare_pose,queue_size=1)
        rospy.Subscriber("/Published_pose",PoseStamped,self.New_input, queue_size=1)
        
        
    def New_input(self,Pose):
        Waypoints = self.Waypoints
        transform = self.tf_buffer.lookup_transform("base_link", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        np_b = tf2_geometry_msgs.do_transform_pose(Pose, transform) # New Waypoint in base
        npd = math.sqrt((np_b.pose.position.x)**2+(np_b.pose.position.y)**2)

        if npd > self.distance_threshold:

            transform_np = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            np_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_np)
            np_m.pose.position.z = 0

            if Waypoints == []:
                self.Waypoints.append(np_m)
                #self.pub_goal.publish(self.Waypoints[0])
            else: 
                #transform_np   = self.tf_buffer.lookup_transform("map", Pose.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
                lp_m = Waypoints[-1]
                wpd = math.sqrt((np_m.pose.position.x-lp_m.pose.position.x)**2+(np_m.pose.position.y-lp_m.pose.position.y)**2)
                if wpd > self.distance_new:
                    self.Waypoints.append(np_m)

    def Compare_pose(self,Pose):
        # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
        if len(self.Waypoints) == 0:
            pass
        else:

            # Calculate the distance to all current waiting waypoints
            d2pb = []
            transform_bm = self.tf_buffer.lookup_transform(self.Waypoints[0].header.frame_id,Pose.header.frame_id, Pose.header.stamp, rospy.Duration(1.0))
            transform_mb = self.tf_buffer.lookup_transform(Pose.header.frame_id,self.Waypoints[0].header.frame_id, Pose.header.stamp, rospy.Duration(1.0))
            for Point_m in self.Waypoints:
                Point_b = tf2_geometry_msgs.do_transform_pose(Point_m, transform_mb)
                d2pb.append(math.sqrt(Point_b.pose.position.x**2+Point_b.pose.position.y**2))
            minpos = d2pb.index(min(d2pb))
            
            # If any waypoints in the list is closer to the base discard all waypoints up to the closest point 
            if minpos > 0:
                for i in range(0,minpos):
                    del self.Waypoints[0]
            # If distance to the current first Waypoint is less than the threshhold
            Goal_m      = self.Waypoints[0]
            Goal_b    = tf2_geometry_msgs.do_transform_pose(Goal_m, transform_mb)
            distance2waypoint = math.sqrt(Goal_b.pose.position.x**2+Goal_b.pose.position.y**2)
            if distance2waypoint < self.distance_threshold:
                if len(self.Waypoints)  == 1:
                    # Calculate the direction to the point from the base and subtract keep distance to that Pose
                    Goal_m_k = 
                    Pose_m = tf2_geometry_msgs.do_transform_pose(Pose, transform_bm)
                    self.pub_goal.publish(Pose_m)
                    del self.Waypoints[0]
                else: 
                    del self.Waypoints[0]
                    self.pub_goal.publish(self.Waypoints[0])


if __name__ == '__main__':
    try:
        Follow()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass