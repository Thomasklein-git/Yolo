#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('/Published_pose', PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/zed2_left_camera_frame"

        pose.pose.position.x    = float(1)
        pose.pose.position.y    = float(0)
        pose.pose.position.z    = float(0)
        pose.pose.orientation.x = float(0)
        pose.pose.orientation.y = float(0)
        pose.pose.orientation.z = float(0)
        pose.pose.orientation.w = float(1)
        
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass