#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

### Imports for ROS ###
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
###

### New tracker
from Frank.Object_handler import Object_handler

### Imports for Yolo
from yolov3.utils import detect_image, Load_Yolo_model#, Give_boundingbox_coor_class
from yolov3.configs import *
from yolov3.yolov3 import *
from agfh import *
### 

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class object_tracker:
	def __init__(self):
		print("[INFO] Loading model...")
		#global yolo
		self.yolo = Load_Yolo_model()
		
		print("[INFO] Loading modules...")
		self.bridge = CvBridge()
		classNum = len(list(read_class_names(YOLO_COCO_CLASSES).values()))
		self.ClassNames = read_class_names(YOLO_COCO_CLASSES)
		self.OH 	= Object_handler(classNum)

		print("[INFO] Loading videofeed...")	
		image_sub = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color",Image)
		#depth_sub = message_filters.Subscriber("/zed2/zed_node/depth/depth_registered",Image)
		cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)

		print("[INFO] initializing config...")
		#self.show=1
		self.seg_plot=True
		#self.dep_active = 0
		#self.cal_active = 0
		#self.min_depth = 0.3
		
		print("[INFO] Initialize Display...")


		Frank = cv2.imread(os.path.join(os.path.dirname( __file__ ),"Frank/Frank.png"),cv2.IMREAD_COLOR)
		#cv2.imshow("Image_window",Frank)
		#detect_image(yolo, Frank, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
		#self.Update_Images()
		print("[INFO] Loading complete")
		#mf = message_filters.ApproximateTimeSynchronizer([image_sub,depth_sub,cloud_sub],1,0.07)
		#mf = message_filters.ApproximateTimeSynchronizer([image_sub,cloud_sub],1,0.07)
		mf = message_filters.TimeSynchronizer([image_sub,cloud_sub],1)
		mf.registerCallback(self.callback)


	#def callback(self,image,depth,cloud):
	def callback(self,image,cloud):
		print("start")
		# Generate images from msgs
		cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
		#cv_image_depth = self.bridge.imgmsg_to_cv2(depth, depth.encoding)
		_,cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
		# Yolo to get Boundary Boxes
		image_detec , bboxes=detect_image(self.yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
		print("detected")
		# Convert Boundary boxes to readable values
		PC_image_bbox_sub_series = Sub_pointcloud(cv_image_pc, bboxes)
		"""
		t3=time.time()
		avg_depth_kmean, segmentation_img_kmeans,_ = k_means_pointcloud(PC_image_bbox_sub_series, bboxes, PC=True, seg_plot=self.seg_plot)
		t4=time.time()
		print(avg_depth_kmean)
		print(t4-t3, "time for kmean")

		t1=time.time()
		avg_depth_dbscan_0_1, segmentation_img_DBSCAN_0_1 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.1,min_samples=50)
		t2=time.time()
		print(avg_depth_dbscan_0_1)
		print(t2-t1,"time for dbscan")
		"""
		#_, segmentation_img_DBSCAN_0_05 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.05,min_samples=50)
		#_, segmentation_img_DBSCAN_0_05 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.05)
		#_, segmentation_img_DBSCAN_0_047 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.047)
		t1=time.time()
		avg_depth_dbscan, segmentation_img_DBSCAN_0_046,_ = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.046)
		t2=time.time()
		print(avg_depth_dbscan)
		print(t2-t1,"time for dbscan")
		#_, segmentation_img_DBSCAN_0_045 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.045)
		#_, segmentation_img_DBSCAN_0_04 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.04)
		#_, segmentation_img_DBSCAN_0_05_80 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.05,min_samples=80)
		#_, segmentation_img_DBSCAN_0_1_100 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.1,min_samples=100)
		#_, segmentation_img_DBSCAN_0_07 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.07)
		#_, segmentation_img_DBSCAN_0_06 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.06)
		#_, segmentation_img_DBSCAN_0_055 = DBSCAN_pointcloud(PC_image_bbox_sub_series,bboxes, seg_plot=self.seg_plot ,eps=0.055)
		x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)

		
		if self.seg_plot==True:
			cv2.imshow("detected img", image_detec)
			for i in range(len(bboxes)):
				#cv2.imshow("segmented_DBSCAN_eps0.1_sam50"+str(0),segmentation_img_DBSCAN_0_1[0])
				#cv2.imshow("segmented_DBSCAN_eps0.1_sam100"+str(0),segmentation_img_DBSCAN_0_1_100[0])
				#cv2.imshow("segmented_DBSCAN_eps0.05_sam50"+str(0),segmentation_img_DBSCAN_0_05[0])
				#cv2.imshow("segmented_DBSCAN_eps0.05_sam80"+str(0),segmentation_img_DBSCAN_0_05_80[0])
				#cv2.imshow("segmented_DBSCAN_eps0.04",segmentation_img_DBSCAN_0_04[0])
				#cv2.imshow("segmented_DBSCAN_eps0.045",segmentation_img_DBSCAN_0_045[0])
				cv2.imshow("segmented_DBSCAN_eps0.046 "+str(i),segmentation_img_DBSCAN_0_046[i])
				#cv2.imshow("segmented_DBSCAN_eps0.047",segmentation_img_DBSCAN_0_047[0])
				#cv2.imshow("segmented_DBSCAN_eps0.05",segmentation_img_DBSCAN_0_05[0])
				#cv2.imshow("segmented_DBSCAN_eps0.055 "+str(i),segmentation_img_DBSCAN_0_055[i])
				#cv2.imshow("segmented_DBSCAN_eps0.06",segmentation_img_DBSCAN_0_06[0])
				#cv2.imshow("segmented_DBSCAN_eps0.07",segmentation_img_DBSCAN_0_07[0])
				#cv2.imshow("segmented_kmeans"+str(0),segmentation_img_kmeans[0])
			cv2.waitKey(3)
		
		"""
		boxes = []
		for i in range(len(bboxes)):	
			boxes.append([x1[i],y1[i],x2[i],y2[i],Score[i],C[i],xyzcoord_series[i]])
		boxes = np.array(boxes)	
		self.OH.add(boxes)
		self.show_img(cv_image)
		"""

	def show_img(self,image):
		for Object in self.OH.Known:
			if Object[self.OH.KnownOrder.get("Occlusion")] <= 5:
				cv2.rectangle(image, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]), \
					(Object[self.OH.KnownOrder.get("End_x")], Object[self.OH.KnownOrder.get("End_y")]),(0, 255, 0), 2)
				text = "Class: {}, ID {}".format(self.ClassNames.get(Object[self.OH.KnownOrder.get("Class")]),Object[self.OH.KnownOrder.get("ID")])
				cv2.putText(image, text, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				cv2.circle(image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (0, 255, 0), -1)
				Position = "X: {}, Y: {}, Z: {}".format(round(Object[self.OH.KnownOrder.get("Depth_X")],2),round(Object[self.OH.KnownOrder.get("Depth_Y")],2),round(Object[self.OH.KnownOrder.get("Depth_Z")],2))
				cv2.putText(image, Position, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]-5),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
				
			else:
				cv2.circle(image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (255, 0, 0), -1)

		cv2.imshow("Image_window", image)
		cv2.waitKey(1)

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
