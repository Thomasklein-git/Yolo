#!/usr/bin/env python3

import numpy as np
import sensor_msgs.point_cloud2 as pc2
import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Image_grabber():
    def __init__(self):
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)
        image_sub = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color",Image)
        #self.depth_sub = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
        depth_sub = message_filters.Subscriber("/zed2/zed_node/depth/depth_registered",Image)
        #self.cloud_sub = rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,self.callback_cloud)
        cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub,depth_sub,cloud_sub],1,10)
    
        ts.registerCallback(self.callback)
        print("Init")
    
    def callback(self,image,depth,cloud):
        print("hej")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
            cv_depth = self.bridge.imgmsg_to_cv2(depth, depth.encoding)
            cv2.imshow("Image",cv_image)
            cv2.imshow("Depth",cv_depth)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)
  
def main(args):
    IG = Image_grabber()
    
    rospy.init_node('Depth_Comparison', anonymous=True)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows

if __name__ =='__main__':
    main(sys.argv)