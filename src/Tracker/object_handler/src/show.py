#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class show:
    def __init__(self):
        rospy.init_node('Draw_boxes')

        self.bridge = CvBridge()

        print("[INFO] Loading videofeed...")	
        image_sub = message_filters.Subscriber("/zed2/zed_node/left/image_rect_color/compressed",CompressedImage, queue_size=1)
        boxed_sub = message_filters.Subscriber("/Tracker/Object_Tracker/Boxes", Detection2DArray, queue_size=1)

        self.image_pub = rospy.Publisher("/Push/Detected_Image",Image,queue_size=1)

        mf = message_filters.TimeSynchronizer([image_sub,boxed_sub],50)
        mf.registerCallback(self.callback)

    def callback(self,image,boxes):
        print("Image pub")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")

        for box in boxes.detections:
            if box.is_tracking == True:
                color = (0,0,255)
            else:
                color = (0,255,0)

            Start_x = box.bbox.center.x-box.bbox.size_x/2
            End_x   = box.bbox.center.x+box.bbox.size_x/2
            Start_y = box.bbox.center.y-box.bbox.size_y/2
            End_y   = box.bbox.center.y+box.bbox.size_y/2

            Class   = box.results[0].id
            Score   = box.results[0].score
            UID     = box.UID
            
            Nd = 2
            Posx       = str(round(box.results[0].pose.pose.position.x, Nd))
            Posy       = str(round(box.results[0].pose.pose.position.y, Nd))
            Posz       = str(round(box.results[0].pose.pose.position.z, Nd))

            Position    = "X: {}, Y: {}, Z: {}".format(Posx,Posy,Posz)
            Description = "Class: {}, ID {}".format(Class, UID)
            
            cv2.rectangle(cv_image, (int(Start_x),int(Start_y)), (int(End_x),int(End_y)), color, 1)
            
            cv2.putText(cv_image, Position,     (int(box.bbox.center.x), int(box.bbox.center.y)-5), cv2.FONT_HERSHEY_PLAIN, 1, color, 2)
            cv2.putText(cv_image, Description,  (int(Start_x), int(End_y)-5),                                 cv2.FONT_HERSHEY_PLAIN, 1, color, 2)
				

        boxed_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(boxed_image)
                

def main(args):
	ot = show()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)