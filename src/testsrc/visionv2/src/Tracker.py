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

### Imports for first tracking model
#from trackutils import cvdraw
from pyimagesearch.centroidtracker import CentroidTracker
###

### Imports for Yolo
from yolov3.utils import detect_image, Load_Yolo_model, Give_boundingbox_coor_class
from yolov3.configs import *
### 

os.environ['CUDA_VISIBLE_DEVICES'] = '0'

class object_tracker:
	def __init__(self):
		print("[INFO] Loading modules...")
		self.bridge = CvBridge()
		#self.cvd = cvdraw()
		self.ct = CentroidTracker()

		print("[INFO] Loading videofeed...")
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

		print("[INFO] Loading model...")
		global yolo
		yolo = Load_Yolo_model()
		#self.net = cv2.dnn.readNetFromCaffe(os.path.join(os.path.dirname(__file__),prototxt),os.path.join(os.path.dirname(__file__),model))
		print("[INFO] Loading complete")

	def callback(self,data):
		try:
			W, H = YOLO_INPUT_SIZE, YOLO_INPUT_SIZE
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
			#cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
			
			###############
			cv_image2, bboxes=detect_image(yolo, cv_image, "", input_size=YOLO_INPUT_SIZE, show=False, rectangle_colors=(255,0,0))
			x1, y1, x2, y2, Score, C = Give_boundingbox_coor_class(bboxes)
			rects = []
			for i in range (0,len(x1)):
				rects.append(np.array([x1[i],y1[i],x2[i],y2[i]],dtype=int))
				#box = np.array(x1[i],y1[i],x2[i],y2[i])
				#rects.append(box.astype("int"))

			###############
		#	frame = cv2.resize(cv_image, (W,H))
		#	blob = cv2.dnn.blobFromImage(frame,1.0, (W,H),(104.0,177.0,123.0))
		#	self.net.setInput(blob)
		#	detections = self.net.forward()
		#	rects = []

		#	for i in range(0, detections.shape[2]):
		#		if detections[0,0,i,2] > [0.5]:
		#			box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
		#			rects.append(box.astype("int"))
		#			(startX, startY, endX, endY) = box.astype("int")
		#			cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 255, 0), 2)  

			objects = self.ct.update(rects)
			for (objectID, centroid) in objects.items():
				text = "ID {}".format(objectID)
				cv2.putText(cv_image2, text, (centroid[0] - 10, centroid[1] - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				cv2.circle(cv_image2, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
			################

			cv2.imshow("Image_window", cv_image2)
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
