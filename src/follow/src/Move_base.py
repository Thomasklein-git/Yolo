#!/usr/bin/env python3

import rospy
from tf import TransformBroadcaster, TransformListener

from geometry_msgs.msg import PoseStamped

class Move_base():
    def __init__(self): 
        rospy.init_node('Base_Mover', anonymous=True)
        Pose = PoseStamped()
        Pose.header.stamp = rospy.Time.now()
        Pose.header.frame_id = "/base"
        Pose.pose.position.x    = float(0)
        Pose.pose.position.y    = float(0)
        Pose.pose.position.z    = float(0)
        Pose.pose.orientation.x = float(0)
        Pose.pose.orientation.y = float(0)
        Pose.pose.orientation.z = float(0)
        Pose.pose.orientation.w = float(1)
        self.tfb = TransformBroadcaster()
        #rospy.Subscriber("/Current_goal",PoseStamped,self.New_input, queue_size=1)
        trans = Pose.pose.position
        rot   = Pose.pose.orientation
        self.tfb.sendTransform((trans.x,trans.y,trans.z),(rot.x,rot.y,rot.z,rot.w),rospy.Time.now(),"BAM","map")
        print("passed")

if __name__ == '__main__':
    try:
        Move_base()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

"""

def talker(n):
    pub = rospy.Publisher('/Published_pose', PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        n += 0.4
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "/zed2_left_camera_frame"
        pose.pose.position.x    = float(n)
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
    n = 0
    try:
        talker(0)
    except rospy.ROSInterruptException:
        pass
"""