#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
import sys
from sensor_msgs.msg import Image


class Simple_Image():
    def __init__(self):
        self.bridge = CvBridge()
        self.Topic1_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback_Topic1)

    def callback_Topic1(self,data):
        #print("Not working")
        try:
            self.Image1 = self.bridge.imgmsg_to_cv2(data, data.encoding)
            cv2.imshow("Hurtig Jakob", self.Image1)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)

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