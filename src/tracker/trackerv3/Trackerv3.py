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
from sensor_msgs.msg import CompressedImage
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
		
		self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage,self.callback_cam)
		self.image_sub_depth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered/compressed",CompressedImage,self.callback_depth)
		
		print("[INFO] initializing config")
		self.show=1
		self.active=0
		self.cv_image_cam = []
		self.cv_image_depth = []
		self.cbca = 0
		self.cbda = 0 

		print("[INFO] Loading complete")

	
	def callback_cam(self,data):
		try:
			np_arr = np.fromstring(data.data, np.uint8)
			self.cv_image_cam = cv2.imdecode(np_arr, cv2.COLOR_BGR2RGB)
			#self.cv_image_depth = self.bridge.compressed_imgmsg_to_cv2(data, cv2.)
			self.cbca = 1
		except CvBridgeError as e:
			print(e)
		
		# If no new depth image and image is generated don't run, 
		#if self.active==0 and self.cbca == 1 and self.cbda == 1:
		#	self.calculation()
		#	self.show_img()
		if self.active==0 and self.cbca == 1 and self.cbda == 1:
			self.calculation()
			self.show_img()
		

	def callback_depth(self,data):
		try:
			self.cv_image_depth = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
			self.cbda = 1
		except CvBridgeError as e:
			print(e)

		

	def calculation(self):
		self.active = 1
		imagecv_cam=self.cv_image_cam
		self.Current_image = imagecv_cam
		imagecv_depth=self.cv_image_depth
		imagecv_cam, bboxes=detect_image(yolo, self.cv_image_cam, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
		x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)
		imagecv_depth_series=[]
		boxes = []
		for i in range(len(bboxes)):
			patch=(int(x2[i]-x1[i]),int(y2[i]-y1[i])) # gives width and height of bbox
			center=(int(x1[i]+patch[0]/2),int(y1[i]+patch[1]/2)) # gives center coodintes of bbox global
			cv_image_bbox_sub = cv2.getRectSubPix(imagecv_depth,patch,center) # Extract bbox in depth image
			cv_image_bbox_sub = np.where(np.isnan(cv_image_bbox_sub),0.3, cv_image_bbox_sub) # set nan to 0
			cv_image_bbox_sub = np.where(np.isinf(cv_image_bbox_sub),20, cv_image_bbox_sub) # set nan to 0
			Distance_to_center_of_bbox_wrt_local=cv_image_bbox_sub[int(patch[1]/2),int(patch[0]/2)] #height (y), width (x) gives distance to center coordinate of bbox
		
			boxes.append([x1[i], y1[i], x2[i], y2[i], Score[i], C[i], Distance_to_center_of_bbox_wrt_local])
		boxes = np.array(boxes)
		self.OH.add(boxes)

		self.active=0
		self.cbca = 0
		self.cbda = 0


	def show_img(self):
		#cv_image = self.Current_image
		cv_image = self.cv_image_cam
		for Object in self.OH.Known:
			if Object[self.OH.KnownOrder.get("Occlusion")] <= 5:
				cv2.rectangle(cv_image, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]), \
					(Object[self.OH.KnownOrder.get("End_x")], Object[self.OH.KnownOrder.get("End_y")]),(0, 255, 0), 2)
				text = "Class: {}, ID {}".format(self.ClassNames.get(Object[self.OH.KnownOrder.get("Class")]),Object[self.OH.KnownOrder.get("ID")])
				cv2.putText(cv_image, text, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.circle(cv_image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (0, 255, 0), -1)
				Position = "X: {}, Y: {}, Z: {}".format(round(Object[self.OH.KnownOrder.get("Depth_X")],2),round(Object[self.OH.KnownOrder.get("Depth_Y")],2),round(Object[self.OH.KnownOrder.get("Depth_Z")],2))
				cv2.putText(cv_image, Position, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				
			else:
				cv2.circle(cv_image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (255, 0, 0), -1)

		cv2.imshow("Image_window", cv_image)
		cv2.waitKey(3)

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
