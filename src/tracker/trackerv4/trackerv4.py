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
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import message_filters
###

### New tracker
#from Frank.Object_handler import Object_handler
from Object_handler_test_vel import Object_handler

### Imports for Yolo
from yolov3.utils import detect_image, Load_Yolo_model
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
		self.pose = PoseStamped()

		print("[INFO] Loading videofeed...")	
		image_sub = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color",Image)
		#depth_sub = message_filters.Subscriber("/zed2/zed_node/depth/depth_registered",Image)
		cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)
		self.pose_pub = rospy.Publisher('/Published_pose', PoseStamped, queue_size=1)

		print("[INFO] initializing config...")
		self.show = False # Show tracker
		self.seg_plot = False # Create segmentation plot
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
		#mf = message_filters.ApproximateTimeSynchronizer([image_sub,cloud_sub],1,0.07) #Set close to zero in order to syncronize img and point cloud (be aware of frame rate) 
		mf = message_filters.TimeSynchronizer([image_sub,cloud_sub],1)
		mf.registerCallback(self.callback)

	#def callback(self,image,depth,cloud):
	def callback(self,image,cloud):
		print("start")
		Time = float("%.6f" %  image.header.stamp.to_sec()) # get time stamp for image in callback
		# Generate images from msgs
		cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
		#cv_image_depth = self.bridge.imgmsg_to_cv2(depth, depth.encoding)
		cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
		# Yolo to get Boundary Boxes
		_ , bboxes=detect_image(self.yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
		# Convert Boundary boxes to readable values
		PC_image_bbox_sub_series = Sub_pointcloud(cv_image_pc, bboxes)
		avg_depth, segmentation_img, xyzcoord_series = k_means_pointcloud(PC_image_bbox_sub_series, bboxes, PC=True, seg_plot=self.seg_plot)


		x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)
		boxes = []
		for i in range(len(bboxes)):	
			#boxes.append([x1[i],y1[i],x2[i],y2[i],Score[i],C[i],xyzcoord_series[i]])
			boxes.append([x1[i],y1[i],x2[i],y2[i],Score[i],C[i],xyzcoord_series[i],Time])
		boxes = np.array(boxes)	
		self.OH.add(boxes)
		if self.OH.Known[0][self.OH.KnownOrder.get("UID")] == 0:
			self.pose.header.stamp = rospy.Time.now()
			self.pose.header.frame_id = "zed2_left_camera_frame"
			self.pose.pose.position.x = self.OH.Known[0][self.OH.KnownOrder.get("Depth_X")]
			self.pose.pose.position.y = self.OH.Known[0][self.OH.KnownOrder.get("Depth_Y")]
			self.pose.pose.position.z = self.OH.Known[0][self.OH.KnownOrder.get("Depth_Z")]
			self.pose.pose.orientation.x = float(0)
			self.pose.pose.orientation.y = float(0)
			self.pose.pose.orientation.z = float(0)
			self.pose.pose.orientation.w = float(1)
			self.pose_pub.publish(self.pose)
		
		if self.show == True:
			self.show_img(cv_image,segmentation_img)

	def show_img(self,image, segmented_image):
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

		if self.seg_plot==True:
			for i in range(len(segmented_image)):
				cv2.imshow("segmented "+str(i),segmented_image[i])
			cv2.waitKey(3)


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