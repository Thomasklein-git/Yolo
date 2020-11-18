#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
import math
import numpy as np




class Driver:
    def __init__(self):
        print("[INFO] Initializing ROS...")
        rospy.init_node('Driver')

        print("[INFO] Loading modules...")


        print("[INFO] Loading config...")
        self.Current_goal = PoseStamped()
        self.Current_goal.pose.orientation.w = float(1)
        step     = 0.05


        print("[INFO] Initialize VPose...")
        Vpose = Odometry()
        Vpose.header.stamp = rospy.Time.now()
        Vpose.header.frame_id = "map"
        Vpose.pose.pose.position.x    = float(0)
        Vpose.pose.pose.position.y    = float(0)
        Vpose.pose.pose.position.z    = float(0)
        Vpose.pose.pose.orientation.x = float(0)
        Vpose.pose.pose.orientation.y = float(0)
        Vpose.pose.pose.orientation.z = float(0)
        Vpose.pose.pose.orientation.w = float(1)

        print("[INFO] Initialize ROS publishers...")
        self.current_goal_pub = rospy.Publisher("/move_base/Current_goal",PoseStamped,queue_size=1)
        self.base_pub = rospy.Publisher('/odometry/filtered_map', Odometry, queue_size=1)
        

        print("[INFO] Initialize ROS Subscribers...")
        rospy.Subscriber("/move_base_simple/goal",PoseStamped,self.cb_current_pose,queue_size=1)


        print("[INFO] Initialize Transform...")
        self.br = TransformBroadcaster()
        trans = (0,0,0)
        rot   = (0,0,0,1)
        stamp = rospy.Time.now()
        self.br.sendTransform(trans, rot, stamp,"base_link","map")
        
        print("[INFO] Loading complete")
        rate = rospy.Rate(5) # Hz
        
        while not rospy.is_shutdown():

            VPose = Odometry()
            Movex = self.Current_goal.pose.position.x - Vpose.pose.pose.position.x
            Movey = self.Current_goal.pose.position.y - Vpose.pose.pose.position.y
            Movez = self.Current_goal.pose.position.z - Vpose.pose.pose.position.z
            Movec = [Movex,Movey,Movez]
            Movemag = np.linalg.norm([Movex,Movey,Movez])
            if Movemag == 0:
                #print("At goal, rotating if needed")
                Move = [0,0,0]
            elif Movemag < step:
                #print("Close, moving to goal")
                Move = [Movex, Movey, Movez]
            else:
                #print("Far away, moving towards goal")
                Move = ([Movex,Movey,Movez]/Movemag)*step
            Vpose.header.stamp = rospy.Time.now()
            Vpose.pose.pose.position.x += Move[0]
            Vpose.pose.pose.position.y += Move[1]
            Vpose.pose.pose.position.z += Move[2]

            self.br = TransformBroadcaster()
            trans = (Vpose.pose.pose.position.x, Vpose.pose.pose.position.y, Vpose.pose.pose.position.z)
            rot   = (Vpose.pose.pose.orientation.x, Vpose.pose.pose.orientation.y, Vpose.pose.pose.orientation.z, Vpose.pose.pose.orientation.w)
            self.br.sendTransform(trans, rot, Vpose.header.stamp,"base_link","map") #Pose.header.stamp,"base","map")
            self.base_pub.publish(Vpose)
            #print("Moved", Move)


            rate.sleep()

    def cb_current_pose(self,Pose):
        self.Current_goal = Pose
        self.current_goal_pub.publish(Pose)




if __name__ == '__main__':
    try:
        Driver()
    except rospy.ROSInterruptException:
        pass