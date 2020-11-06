#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

### Imports for ROS ###
#from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField
from vision_msgs.msg import Detection2DArray
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseStamped
#from cv_bridge import CvBridge, CvBridgeError
import message_filters

from agfh import *

from yolov3.utils import detect_image, Load_Yolo_model
from yolov3.configs import *
from yolov3.yolov3 import *

### New tracker ###
from Object_handler import Object_handler


#os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class object_tracker:
    def __init__(self):
        #print("[INFO] Loading modules...")
        classNum        = len(list(read_class_names(YOLO_COCO_CLASSES).values()))
        self.ClassNames = read_class_names(YOLO_COCO_CLASSES)
        self.OH 	    = Object_handler(classNum)

        print("[INFO] initializing config...")
        self.Target_class = 0 # Class 0 is person
        self.Target_Found = False
        self.Target_UID = []

        #print("[INFO] Loading ROS topics")
        self.Tracking_list = rospy.Publisher("/yolo/Trackedbboxes", Detection2DArray, queue_size=1)
        rospy.Subscriber("/yolo/Segbboxes", Detection2DArray, self.callback, queue_size=1)

        print("[INFO] Loading complete")

    def callback(self,boxes):

        boxes_OH = box_for_OH(boxes)
        print(boxes_OH)
            

        pass

        #self.Tracking_list.publish(boxes)

def box_for_OH(boxes):
        # Takes boxes in the format of Detection2DArray.msg and converts it to fit the format Object_handler
        boxes_OH = []
        for box in boxes.detections:
            cx = box.bbox.center.x # Center x
            sx = box.bbox.size_x   # Size x
            cy = box.bbox.center.y # Center y
            sy = box.bbox.size_y   # Size y

            Start_x = int(cx - sx/2)
            End_x   = int(cx + sx/2)
            Start_y = int(cy - sy/2)
            End_y   = int(cy + sy/2)
            Score   = box.results[0].score
            Class   = box.results[0].id
            boxes_OH.append([Start_x,End_x,Start_y,End_y,Score,Class])
        boxes_OH = np.array(boxes_OH)
        return boxes_OH

def main(args):
	rospy.init_node('object_tracker', anonymous=True)
	ot = object_tracker()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)
