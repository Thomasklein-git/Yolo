#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

### Imports for ROS ###
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
###

### New tracker
from Frank.Object_handler import Object_handler

### Imports for Yolo
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *
from yolov3.yolov3 import *
from agfh import *
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
		self.image_sub_depth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered",Image,self.callback_depth)
		self.image_sub_camera = rospy.Subscriber("/zed2/zed_node/left/image_rect_color",Image,self.callback_cam)

		print("[INFO] initializing config...")
		self.show=1
		self.dep_active = 0
		self.cal_active = 0

		#self.Update_Images()
		print("[INFO] Loading complete")

	def callback_cam(self,data):
		if self.dep_active == 1:
			self.cal_active = 1
			try:
				self.cv_image_cam = self.bridge.imgmsg_to_cv2(data, data.encoding)
			except CvBridgeError as e:
				print(e)

			imagecv_cam,cv_image_bbox_sub,bboxes,img_seg=self.calculation()	
			
			self.cal_active=0
			self.dep_active = 0

	def callback_depth(self,data):
		if self.cal_active ==0:
			try:
				self.cv_image_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
			except CvBridgeError as e:
				print(e)
			self.dep_active=1

	def calculation(self):
		imagecv_cam=self.cv_image_cam
		imagecv_depth=self.cv_image_depth
		imagecv_depth_series=[]
		img_seg=[]
		#if imagecv_cam != []:
		if len(imagecv_cam)  != 0:
			imagecv_cam, bboxes=detect_image(yolo, imagecv_cam, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
			x1, y1, x2, y2, _, C = Give_boundingbox_coor_class(bboxes)
			#print("Bounding box of object(s) = ",x1,y1,x2,y2,C)
		if len(imagecv_depth) != 0:
			for i in range(len(bboxes)):
				patch=(int(x2[i]-x1[i]),int(y2[i]-y1[i])) # gives width and height of bbox
				center=(int(x1[i]+patch[0]/2),int(y1[i]+patch[1]/2)) # gives center coodintes of bbox global
				cv_image_bbox_sub = cv2.getRectSubPix(imagecv_depth,patch,center) # Extract bbox in depth image
				cv_image_bbox_sub = np.where(np.isnan(cv_image_bbox_sub),0, cv_image_bbox_sub) # set nan to 0
				cv_image_bbox_sub = np.where(np.isinf(cv_image_bbox_sub),0, cv_image_bbox_sub) # set +/-inf to 0
				avg_depth,img_seg=k_means_depth(cv_image_bbox_sub)
				D_to_C_of_bbox_L=cv_image_bbox_sub[int(patch[1]/2),int(patch[0]/2)] #height (y), width (x) gives distance to center coordinate of bbox with resprct to local
				imagecv_depth_series.append(cv_image_bbox_sub)
		return imagecv_cam, imagecv_depth_series, bboxes, img_seg
		"""
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
			
			max_depth=20 #Maximum depth the camera can detect objects [m]
        	cv_image_bbox_sub *= 255/(max_depth/cv_image_bbox_sub) # relate meter to pixels
        	cv_image_bbox_sub = cv_image_bbox_sub.astype('uint8') # pixel float to int
        	Distance_to_center_of_bbox_wrt_local_pixel=cv_image_bbox_sub[int(patch[1]/2),int(patch[0]/2)]
			
		
		
		#for i in range(0,len(x1)):
			boxes.append([x1[i], y1[i], x2[i], y2[i], Score[i], C[i], Distance_to_center_of_bbox_wrt_local])
		boxes = np.array(boxes)
		self.OH.add(boxes)

		#imagecv_depth_series.append(cv_image_bbox_sub)
		
		
		cv_image = cv_image_cam
		for Object in self.OH.Known:
			if Object[self.OH.KnownOrder.get("Occlusion")] <= 5:
				cv2.rectangle(cv_image, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]), \
					(Object[self.OH.KnownOrder.get("End_x")], Object[self.OH.KnownOrder.get("End_y")]),(0, 255, 0), 2)
				text = "Class: {}, ID {}".format(self.ClassNames.get(Object[self.OH.KnownOrder.get("Class")]),Object[self.OH.KnownOrder.get("ID")])
				cv2.putText(cv_image, text, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.circle(cv_image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (0, 255, 0), -1)
			else:
				cv2.circle(cv_image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (255, 0, 0), -1)

		cv2.imshow("Image_window", cv_image)
		cv2.waitKey(3) 
		"""

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
"""
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
				if Object[self.OH.KnownOrder("Occlusion")] <= 5:
					cv2.rectangle(cv_image, (Object[self.OH.KnownOrder("Start_x")], Object[self.OH.KnownOrder("Start_y")]), \
						(Object[self.OH.KnownOrder("End_x")], Object[self.OH.KnownOrder("End_y")]),(0, 255, 0), 2)
					text = "Class: {}, ID {}".format(self.ClassNames.get(Object[self.OH.KnownOrder("Class")]),Object[self.OH.KnownOrder("ID")])
					cv2.putText(cv_image, text, (Object[self.OH.KnownOrder("Start_x")], Object[self.OH.KnownOrder("Start_y")]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
					cv2.circle(cv_image, (Object[self.OH.KnownOrder("cx")], Object[self.OH.KnownOrder("cy")]), 4, (0, 255, 0), -1)
				else:
					cv2.circle(cv_image, (Object[self.OH.KnownOrder("cx")], Object[self.OH.KnownOrder("cy")]), 4, (255, 0, 0), -1)

			cv2.imshow("Image_window", cv_image)
			cv2.waitKey(3)
		except CvBridgeError as e:
			print(e)
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
