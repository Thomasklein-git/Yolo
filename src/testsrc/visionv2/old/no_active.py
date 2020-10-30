#!/usr/bin/env python3


#### New tracker ####
from Frank.Object_handler import Object_handler
from agfh import *

##### ROS #####
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import rospy

#### pip packages ####
import numpy as np
import sys
import cv2
import os

#### Imports for Yolo ####
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *
from yolov3.yolov3 import *

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class Image_grabber():
    def __init__(self):
        print("[INFO] Loading model...")
        global yolo
        yolo = Load_Yolo_model()
        
        print("[INFO] Loading modules...")
        self.bridge = CvBridge()
        classNum = len(list(read_class_names(YOLO_COCO_CLASSES).values()))
        self.ClassNames = read_class_names(YOLO_COCO_CLASSES)
        self.OH 	= Object_handler(classNum)
        
        print("[INFO] Loading videofeed...")
        image_sub = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color",Image)
        cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)
        mf = message_filters.ApproximateTimeSynchronizer([image_sub,cloud_sub],1,10)
        mf.registerCallback(self.callback)

        print("INFO Initialize display")


        print("[INFO] Loading complete")

    def callback(self,image,cloud):
        # RBG image
        print("running")
        cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
        # Yolo to get Boundary Boxes
        _ , bboxes=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))

        
        
        cv2.imshow("Depth",cv_image)
        cv2.waitKey(3)
        
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
            cv_depth = self.bridge.imgmsg_to_cv2(depth, depth.encoding)
        except CvBridgeError as e:
            print(e)
        pc = pc2.read_points(cloud, skip_nans=False, field_names=("x", "y", "z"))
        
        x = []
        y = []
        z = []
        for p in pc:
            x.append(p[0])
            y.append(p[1])
            z.append(p[2])
        
        #print(x)
        x= np.array(x)
        xnonan = np.where(np.isnan(x), 0, x)
        
        xp = xnonan-np.min(xnonan)
        xp = np.where(np.isnan(x),0,xp)
        xn = (65536*(xp)/np.ptp(xp)).astype(np.uint16) 
        
        xn = xn.reshape(720,1280)
        #print(z[-1], "z")
        #print(cv_depth[-1], "d")
        
        #cv2.imshow("Image",cv_image)
        cv2.imshow("Depth",xn)
        cv2.waitKey(3)
        """

def main(args):
    rospy.init_node('Depth_Comparison', anonymous=True)
    IG = Image_grabber()    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows

if __name__ =='__main__':
    main(sys.argv)