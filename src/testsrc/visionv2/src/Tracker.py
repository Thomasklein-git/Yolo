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
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import message_filters
###

### New tracker
#from Frank.Object_handler import Object_handler
from Object_handler import Object_handler

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

		print("[INFO] Loading videofeed...")	
		image_sub = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color",Image)
		cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)
		#Odometry_sub = message_filters.Subscriber('/odometry/filtered_map', Odometry)
		#self.vehicle_pose = rospy.Subscriber('/odometry/filtered_map', Odometry, self.callback_odom)
		self.pose_pub = rospy.Publisher('/Published_pose', PoseStamped, queue_size=1)
		self.reduc_cloud_pub = rospy.Publisher("/Reduced_cloud", PointCloud2, queue_size=1)
		#self.seg_plot_pub = rospy.Publisher("/seg_img",Image,queue_size=1)

		self.timed_cloud_pub = rospy.Publisher("/Timed_cloud", PointCloud2, queue_size=1)
		self.boxed_image_pub = rospy.Publisher("/Boxed_image", Image, queue_size=1)

		print("[INFO] initializing config...")
		self.Target_class = 0 # Class 0 is person
		self.Target_Found = False
		self.Target_UID = []
		self.show = True # Show tracker
		self.seg_plot = True # Create segmentation plot
		
		print("[INFO] Initialize Display...")
		Frank = cv2.imread(os.path.join(os.path.dirname( __file__ ),"Frank/Frank.png"),cv2.IMREAD_COLOR)

		print("[INFO] Loading complete")
		#mf = message_filters.ApproximateTimeSynchronizer([image_sub,depth_sub,cloud_sub],1,0.07)
		#mf = message_filters.ApproximateTimeSynchronizer([image_sub,cloud_sub],1,5) #Set close to zero in order to syncronize img and point cloud (be aware of frame rate) 
		mf = message_filters.TimeSynchronizer([image_sub,cloud_sub],1)
		mf.registerCallback(self.callback)

	#def callback(self,image,depth,cloud):
	def callback(self,image,cloud):
		Time = float("%.6f" %  cloud.header.stamp.to_sec()) # get time stamp for image in callback
		# Generate images from msgs
		cv_image = self.bridge.imgmsg_to_cv2(image, image.encoding)
		#cv_image_depth = self.bridge.imgmsg_to_cv2(depth, depth.encoding)
		pc_list, cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
		# Yolo to get Boundary Boxes
		_ , bboxes=detect_image(self.yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
		# Convert Boundary boxes to readable values
		PC_image_bbox_sub_series = Sub_pointcloud(cv_image_pc, bboxes)
		_, segmentation_img, xyzcoord_series, labels_series = DBSCAN_pointcloud(PC_image_bbox_sub_series, bboxes, seg_plot=self.seg_plot)

		xyzcoord_trans_series = Transform_Coordinates_between_frames(xyzcoord_series,"zed2_left_camera_frame","map",Time)

		x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)
		boxes = []
		for i in range(len(bboxes)):	
			#boxes.append([x1[i],y1[i],x2[i],y2[i],Score[i],C[i],xyzcoord_series[i]])
			boxes.append([x1[i],y1[i],x2[i],y2[i],Score[i],C[i],xyzcoord_trans_series[i],Time])
		boxes = np.array(boxes)	
		self.OH.add(boxes)
		fp = True
		#segimg = self.bridge.cv2_to_imgmsg(segmentation_img[0], "bgr8")
		#self.seg_plot_pub.publish(segimg)
		# Find UID to a target
		if self.Target_Found == False:
			self.Target_UID, self.Target_Found = Choose_target(self.OH, self.Target_class)

		# If target 
		if self.Target_Found == True:
			Target_I, Target_Occlusion = Find_target(self.OH, self.Target_UID)
			if Target_I == []:
				print("Target is Lost")
			elif Target_Occlusion > 0:
				print("Target was occluded {} frames ago".format(Target_Occlusion))
			else:
				Target = self.OH.Known[Target_I]
				TargetOrder = self.OH.KnownOrder.get
				SegID = Target[self.OH.KnownOrder.get("Current_listing")]

				Seg = labels_series[SegID]
				bbox = boxes[SegID]

				
				
				#Reduced_PC  = PC_reduc_seg()
				Reduced_PC = PC_reduc_seg(bbox, Seg ,pc_list,cloud)

				#Reduced_PC  = PC_reduc(Target, TargetOrder, pc_list, cloud)
				self.reduc_cloud_pub.publish(Reduced_PC)
				self.timed_cloud_pub.publish(cloud)

				Pose 		= Waypoint_planter(Target, TargetOrder, "zed2_left_camera_frame", rospy.Time.now())
				self.pose_pub.publish(Pose)
		else: 
			Reduced_PC  = PC_reduc(None, None, pc_list, cloud)
			self.reduc_cloud_pub.publish(Reduced_PC)
			self.timed_cloud_pub.publish(cloud)

		if self.show == True:
			self.show_img(cv_image,segmentation_img, image)

	def show_img(self,image, segmented_image,imagemsg):
		#print(self.Target_UID, "target")
		for i in range(len(self.OH.Known)):
			#print(i)
			Object = self.OH.Known[i][:]
			#print(Object)
			#print(self.OH.Known,"OH")
			#print(self.OH.Known[:][i],"OH række")
			if Object[self.OH.KnownOrder.get("Occlusion")] <= 5:
				if Object[self.OH.KnownOrder.get("UID")] == self.Target_UID:
					print("target")
					cv2.rectangle(image, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]), \
						(Object[self.OH.KnownOrder.get("End_x")], Object[self.OH.KnownOrder.get("End_y")]),(0, 0, 255), 2)
				else:
				#	print("not target")
					cv2.rectangle(image, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("Start_y")]), \
						(Object[self.OH.KnownOrder.get("End_x")], Object[self.OH.KnownOrder.get("End_y")]),(0, 255, 0), 1)
				text = "Class: {}, ID {}".format(self.ClassNames.get(Object[self.OH.KnownOrder.get("Class")]),Object[self.OH.KnownOrder.get("ID")])
				cv2.putText(image, text, (Object[self.OH.KnownOrder.get("Start_x")], Object[self.OH.KnownOrder.get("End_y")]-5),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
				cv2.circle(image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (0, 255, 0), -1)
				Nd = 3
				
				Posx = str(round(Object[self.OH.KnownOrder.get("Depth_X")], Nd))
				Posy = str(round(Object[self.OH.KnownOrder.get("Depth_Y")], Nd))
				#print(Posy)
				Posz = str(round(Object[self.OH.KnownOrder.get("Depth_Z")], Nd))
				
				#Position = "X: {}, Y: {}, Z: {}".format(str(round(Object[self.OH.KnownOrder.get("Depth_X")], Nd)),str(round(Object[self.OH.KnownOrder.get("Depth_Y")], Nd)),str(round(Object[self.OH.KnownOrder.get("Depth_Z")], Nd)))
				Position = "X: {}, Y: {}, Z: {}".format(Posx,Posy,Posz)
				cv2.putText(image, Position, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]-5),cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)
				
			else:
				cv2.circle(image, (Object[self.OH.KnownOrder.get("cx")], Object[self.OH.KnownOrder.get("cy")]), 4, (255, 0, 0), -1)

		#cv2.imshow("Image_window", image)
		#cv2.waitKey(1)
		Boxed_image = self.bridge.cv2_to_imgmsg(image, imagemsg.encoding)
		Boxed_image.header = imagemsg.header
		self.boxed_image_pub.publish(Boxed_image)

		#if self.seg_plot==True:
		#	for i in range(len(segmented_image)):
		#		cv2.imshow("segmented "+str(i),segmented_image[i])
		#	cv2.waitKey(3)


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
