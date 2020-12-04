#!/usr/bin/env python3

import sys
import rospy
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster


class dynamic_transform:
    def __init__(self):
        rospy.init_node("Dynamic_Transform_odom")
        self.br = TransformBroadcaster()
        rospy.Subscriber("/odometry/filtered_map",Odometry,self.callback,queue_size=1)

    def callback(self, Odom):
        trans = (Odom.pose.pose.position.x, Odom.pose.pose.position.y, Odom.pose.pose.position.z)
        rot   = (Odom.pose.pose.orientation.x, Odom.pose.pose.orientation.y, Odom.pose.pose.orientation.z, Odom.pose.pose.orientation.w)
        #stamp = rospy.Time.now()
        stamp = Odom.header.stamp
        self.br.sendTransform(trans, rot, stamp,"base_link","map")

def main(args):
	dynamic_transform()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)