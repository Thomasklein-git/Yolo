#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

from vision_msgs.msg import Detection2DArray
import message_filters

from agfh import *

#from yolov3.utils import detect_image, Load_Yolo_model
from yolov3.configs import *
from yolov3.yolov3 import *

### New tracker ###
from Object_handler import Object_handler


class object_tracker:
    def __init__(self):
        print("[INFO] Loading modules...")
        classNum        = len(list(read_class_names(YOLO_COCO_CLASSES).values()))
        self.ClassNames = read_class_names(YOLO_COCO_CLASSES)
        self.OH 	    = Object_handler(classNum)

        print("[INFO] initializing config...")
        self.Target_class = 0 # Class 0 is person
        self.Target_Found = False
        self.Target_UID = []
        self.show = False

        print("[INFO] Loading ROS topics")
        self.Tracking_list = rospy.Publisher("/yolo/Trackedbboxes", Detection2DArray, queue_size=1)

        rospy.Subscriber("/yolo/Segbboxes", Detection2DArray, self.callback, queue_size=1)

        print("[INFO] Loading complete")

    def callback(self,boxes):
        Time = float("%.6f" %  boxes.header.stamp.to_sec())
        boxes_OH = box_for_OH(boxes,Time)
        self.OH.add(boxes_OH)

        if self.Target_Found == False:
            self.Target_UID, self.Target_Found = Choose_target(self.OH, self.Target_class)

        if self.Target_Found == True:
            Target_I, Target_Occlusion = Find_target(self.OH, self.Target_UID)
            if Target_I == []:
                print("Target is Lost")
            elif Target_Occlusion > 0:
                print("Target was occluded {} frames ago".format(Target_Occlusion))
            else:
                Target = self.OH.Known[Target_I]
                SegID = Target[self.OH.KnownOrder.get("Current_listing")]
                boxes.detections[SegID].is_tracking = True

                
        
        for Object in self.OH.Known:
            Current_list = Object[self.OH.KnownOrder.get("Current_listing")]
            if np.isnan(Current_list):
                pass
            else:
                boxes.detections[Current_list].UID = str(Object[self.OH.KnownOrder.get("UID")])


        
        self.Tracking_list.publish(boxes)


        if self.show == True:
            self.show_img(cv_image)


def box_for_OH(boxes,Time):
        # Takes boxes in the format of Detection2DArray.msg and converts it to fit the format Object_handler
        boxes_OH = []
        for box in boxes.detections:
            cx = box.bbox.center.x # Center x
            sx = box.bbox.size_x   # Size x
            cy = box.bbox.center.y # Center y
            sy = box.bbox.size_y   # Size y


            Start_x = int(cx - sx/2)
            End_x   = int(cx + sx/2)
            Start_y = int(cy - sy/2)
            End_y   = int(cy + sy/2)
            Score   = box.results[0].score
            Class   = box.results[0].id


            x = box.results[0].pose.pose.position.x
            y = box.results[0].pose.pose.position.y
            z = box.results[0].pose.pose.position.z
            xyz = [x,y,z]

            boxes_OH.append([Start_x,End_x,Start_y,End_y,Score,Class,xyz,Time])
        boxes_OH = np.array(boxes_OH)
        return boxes_OH

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
