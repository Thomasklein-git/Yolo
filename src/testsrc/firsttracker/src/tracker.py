#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os

from std_msgs.msg import String
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError

from trackutils import cvdraw
from  pyimagesearch.centroidtracker import CentroidTracker

class object_tracker:
	def __init__(self):
		model    ="res10_300x300_ssd_iter_140000.caffemodel"
		prototxt ="deploy.prototxt"

		print("[INFO] Loading modules...")
		self.bridge = CvBridge()
		self.cvd = cvdraw()
		self.ct = CentroidTracker()

		print("[INFO] Loading videofeed...")
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

		print("[INFO] Loading model...")
		self.net = cv2.dnn.readNetFromCaffe(os.path.join(os.path.dirname(__file__),prototxt),os.path.join(os.path.dirname(__file__),model))
		print("[INFO] Loading complete")

	def callback(self,data):
		try:
			W, H = 400, 400
			cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
			cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
			###############
			frame = cv2.resize(cv_image, (W,H))
			blob = cv2.dnn.blobFromImage(frame,1.0, (W,H),(104.0,177.0,123.0))
			self.net.setInput(blob)
			detections = self.net.forward()
			rects = []

			for i in range(0, detections.shape[2]):
				if detections[0,0,i,2] > [0.5]:
					box = detections[0, 0, i, 3:7] * np.array([W, H, W, H])
					rects.append(box.astype("int"))
					(startX, startY, endX, endY) = box.astype("int")
					cv2.rectangle(frame, (startX, startY), (endX, endY),(0, 255, 0), 2)  

			objects = self.ct.update(rects)
			for (objectID, centroid) in objects.items():
				text = "ID {}".format(objectID)
				cv2.putText(frame, text, (centroid[0] - 10, centroid[1] - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				cv2.circle(frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)
			################

			cv2.imshow("Image_window", frame)
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
