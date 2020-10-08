#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os

import struct

### Imports for ROS ###
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
###

### New tracker
#from Object_handler import Object_handler

class object_tracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub_camera=rospy.Subscriber("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage,self.callback_cam)
        
        self.image_sub_depth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered/compressed",CompressedImage,self.callback_depth)
    
    def callback_cam(self,data):
        C_array=np.fromstring(data.data,np.uint8)
        self.cv_image_cam=cv2.imdecode(C_array,cv2.COLOR_BGR2RGB)
        #print("Cam")
        #cv2.imshow("Image_window", self.cv_image_cam)
        #cv2.waitKey(3)
    
    def callback_depth(self,data):
        try:
            self.cv_image_depth = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #print(self.cv_image_depth)
        cv2.imshow("Image_window", self.cv_image_depth)
        cv2.waitKey(3)
        """
        depth_fmt, compr_type = data.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        
        depth_header_size = 12
        raw_data = data.data[depth_header_size:]
        depth_img_raw = cv2.imdecode(np.fromstring(raw_data, np.uint8), cv2.IMREAD_UNCHANGED)
        
        if depth_img_raw is None:   
            # probably wrong header size
            raise Exception("Could not decode compressed depth image. You may need to change 'depth_header_size'!")

        if depth_fmt == "16UC1":
            # write raw image data
            print("None")
            #cv2.imwrite(os.path.join(path_depth, "depth_" + str(data.header.stamp) + ".png"), depth_img_raw)
        elif depth_fmt == "32FC1":
            raw_header = data.data[:depth_header_size]
            # header: int, float, float
            [compfmt, depthQuantA, depthQuantB] = struct.unpack('iff', raw_header)
            depth_img_scaled = depthQuantA / (depth_img_raw.astype(np.float32)-depthQuantB)
            # filter max values
            depth_img_scaled[depth_img_raw==0] = 0

            # depth_img_scaled provides distance in meters as f32
            # for storing it as png, we need to convert it to 16UC1 again (depth in mm)
            depth_img_mm = (depth_img_scaled*1000).astype(np.uint16)
            #cv2.imwrite(os.path.join("../", "depth_" + str(data.header.stamp) + ".png"), depth_img_mm)
            depth_img_scaled *= 255/depth_img_scaled.max()
            
        else:
            raise Exception("Decoding of '" + depth_fmt + "' is not implemented!")
        
        cv2.imshow("P",depth_img_scaled)
        cv2.waitKey(3)
        
        #D_array=np.fromstring(data.data,np.uint8)
        #print(D_array)
        #self.cv_image_depth=cv2.imdecode(D_array,cv2.IMREAD_GRAYSCALE)
        #try:
        #    self.cv_image_depth = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        #except CvBridgeError as e:
        #    print(e)
        #print(self.cv_image_depth)
        #cv2.imshow("Image_window", data.data)
        #cv2.waitKey(3)
        """
def main(args):
	ot = object_tracker()
	rospy.init_node('object_tracker', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)
