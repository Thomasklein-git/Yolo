#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import sys
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
#from sensor_msgs import pc2

class Simple_Image():
    def __init__(self):
        self.Topic1_sub = rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,self.callback_Topic1)

    def callback_Topic1(self,data):
        pc = pc2.read_points(data, skip_nans=False, field_names=("x", "y", "z"))
        pc_list = []
        for p in pc:
            pc_list.append( [p[0],p[1],p[2]] )
        print(pc_list[200000])


def main(args):
    SI = Simple_Image()
    
    rospy.init_node('Simple_Image', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows

if __name__ =='__main__':
    main(sys.argv)