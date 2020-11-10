#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

class cloudimage:
    def __init__(self):
        print("[INFO] Loading modules...")
        self.bridge = CvBridge()
        rospy.Subscriber("/yolo/CloudImage", Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/yolo/CloudImageZ", Image, queue_size = 1)

    def callback(self,image):
        time1 = rospy.Time.now().to_sec()
        cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
        time2 = rospy.Time.now().to_sec()
        #print(time2-time1)
        cv_imageZ = cv_image[:,:,2]
        imageZ = self.bridge.cv2_to_imgmsg(cv_imageZ, "32FC1")
        self.pub.publish(imageZ)

        
        

def main(args):
	rospy.init_node('OBJ', anonymous=True)
	ot = cloudimage()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)