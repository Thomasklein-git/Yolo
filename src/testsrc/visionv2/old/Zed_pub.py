#!/usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import threading

class Queue_Limit:
    def __init__(self):
        self.bridge = CvBridge()

        #self.camera_pub = rospy.Publisher("/Frank/ZED/Camera",CompressedImage,  queue_size=1)
        #self.depth_pub = rospy.Publisher("/Frank/ZED/Depth",CompressedImage,  queue_size=1)

        self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage,self.callback_cam)
        #self.image_sub_depth  = rospy.Subscriber("/zed2/zed_node/depth/depth_registered/compressed",CompressedImage,self.callback_depth)

    def callback_cam(self,data):
        try:
			#self.cv_image_cam = self.bridge.imgmsg_to_cv2(data, data.encoding)
            np_arr = np.fromstring(data.data, np.uint8)
            self.cv_image_cam = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)
			#self.cbca = 1
			#print("Camera")
        except CvBridgeError as e:
            print(e)
        #self.camera_pub.publish(data)
        cv2.imshow("Camera",self.cv_image_cam)
        cv2.waitKey(3)

    def callback_depth(self,data):
        self.depth_pub.publish(data)

    def image_show(self):
        #if self.cam == 1 and self.depth == 1:
            cv2.imshow("Camera",self.cv_image_cam)
            #cv2.imshow("Depth",self.cv_image_depth)
            cv2.waitKey(3)




def main(args):
	QL = Queue_Limit()
	rospy.init_node('Queue_Limit', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)