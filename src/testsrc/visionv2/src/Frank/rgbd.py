#!/usr/bin/env python3
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image
import cv2

class rgbd():
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_pub = rospy.Publisher("/RGBD/ZED2/Camera",Image,  queue_size=1)
        self.depth_pub = rospy.Publisher("/RGBD/ZED2/Depth",Image,  queue_size=1)
        self.image_sub_depth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
        self.image_sub_cam = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)

    def callback_depth(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        cv_image = cv.resize(cv_image,(416,416))
        #print(data.encoding)
        pub_image = self.bridge.cv2_to_imgmsg(cv_image, data.encoding)
        self.depth_pub.publish(pub_image)

    def callback_cam(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        cv_image = cv.resize(cv_image,(416,416))
        pub_image = self.bridge.cv2_to_imgmsg(cv_image, data.encoding)
        self.camera_pub.publish(pub_image)

def main(args):
    rospy.init_node('RGBD', anonymous=True)
    RGBD = rgbd()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ =='__main__':
    main(sys.argv)