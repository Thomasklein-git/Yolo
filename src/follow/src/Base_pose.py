#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped

class Base_pose():
    def __init__(self):
        rospy.init_node('Base_position', anonymous=True)
        rate = rospy.Rate(1) # 1hz
        self.base_pub = rospy.Publisher('/Published_pose', PoseStamped, queue_size=1)
        


if __name__ == '__main__':
    try:
        Base_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass