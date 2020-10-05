#!/usr/bin/env python3

# This scipt is responsible for drawing a box on the image
# corresponding to a boundary box.
import cv2
import numpy


class cvdraw():
	def __init__(self):
		self.x = [0]
		self.y = [0]
		self.w = [200]
		self.h = [200]
		self.color = [0, 255, 0]
		self.font = (cv2.FONT_HERSHEY_SIMPLEX,1,1,0,1)

	def addbox(self,image):
		image2 = cv2.rectangle(image,[100, 100, 200, 200], self.color, thickness = 2)
		return image2

	def addtext(self, image, text):
	#	cv2.putText(image, text,(100, 100),self.font,255)
		cv2.putText(image, text,(100, 100),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0),1)
		return image
