#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os

### Imports for ROS ###
from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
###

### New tracker
from Object_handler import Object_handler

### Imports for Yolo
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *
from yolov3.yolov3 import *
### 

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class object_tracker:
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
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

		print("[INFO] Loading complete")

	def callback(self,data):
		try:
			W, H = YOLO_INPUT_SIZE, YOLO_INPUT_SIZE
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

			cv_image2, bboxes=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
			x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)

			boxes = []
			for i in range(0,len(x1)):
				boxes.append([x1[i], y1[i], x2[i], y2[i], Score[i], C[i]])
			boxes = np.array(boxes)
			self.OH.add(boxes)

			for Object in self.OH.Known:
				if Object[10] <= 5:
					cv2.rectangle(cv_image, (Object[5], Object[6]), (Object[7], Object[8]),(0, 255, 0), 2)
					text = "Class: {}, ID {}".format(self.ClassNames.get(Object[2]),Object[1])
					cv2.putText(cv_image, text, (Object[5], Object[6]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
					cv2.circle(cv_image, (Object[3], Object[4]), 4, (0, 255, 0), -1)
				else:
					cv2.circle(cv_image, (Object[3], Object[4]), 4, (255, 0, 0), -1)

			cv2.imshow("Image_window", cv_image)
			cv2.waitKey(3)
		except CvBridgeError as e:
			print(e)

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
