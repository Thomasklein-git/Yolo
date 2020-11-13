#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

from sensor_msgs.msg import TimeReference
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

import message_filters

from agfh import *

#from yolov3.utils import detect_image, Load_Yolo_model
from yolov3.configs import *
#from yolov3.yolov3 import read_class_names

### New tracker ###
from Object_handler import Object_handler


class object_tracker:
    def __init__(self):
        print("[INFO] Initializing ROS...")
        rospy.init_node('Object_Tracker')

        print("[INFO] Loading modules...")
        classNum        = len(list(read_class_names(YOLO_COCO_CLASSES).values()))
        self.ClassNames = read_class_names(YOLO_COCO_CLASSES)
        self.OH 	    = Object_handler(classNum)

        print("[INFO] initializing config...")
        self.Target_class = 0 # Class 0 is person
        self.Target_Found = False
        self.Target_UID = []

        print("[INFO] Initialize ROS publisher...")
        self.Tracking_list = rospy.Publisher("/Tracker/Object_Tracker/Boxes", Detection2DArray, queue_size=1)
        self.pose_pub = rospy.Publisher('/Tracker/Object_Tracker/Published_pose', PoseStamped, queue_size=1)

        print("[INFO] Initialize ROS Subscribers...")
        #rospy.Subscriber("/yolo/Segbboxes", Detection2DArray, self.callback, queue_size=1)
        boxes_sub = message_filters.Subscriber("/Tracker/Segmentation/Boxes", Detection2DArray, queue_size=1)
        timer_sub = message_filters.Subscriber("/Tracker/Timer", TimeReference, queue_size=1)
        print("[INFO] Loading complete")

        mf = message_filters.TimeSynchronizer([boxes_sub,timer_sub],queue_size=40)
        mf.registerCallback(self.callback)

    def callback(self,boxes,timer):
        Time = float("%.6f" %  boxes.header.stamp.to_sec())
        boxes_OH = box_for_OH(boxes,Time)
        # Coordinates from BB to other coordinate set
        #xyzcoord_trans_series = Transform_Coordinates_between_frames(xyzcoord_series,"zed2_left_camera_frame","map",Time)
        self.OH.add(boxes_OH)

        # If there are no current or previous target, run Choose_target to search for a new target
        if self.Target_Found == False:
            self.Target_UID, self.Target_Found = Choose_target(self.OH, self.Target_class)
        # If there are a current or previous target run find target to search for the target in the current boxes
        if self.Target_Found == True:
            TargetList = Find_target(self.OH, self.Target_UID)
        else:
            TargetList = []

        for Object in self.OH.Known:
            # Search Known Objects for their number in the current bbox list
            Current_list = Object[self.OH.KnownOrder.get("Current_listing")]
            if np.isnan(Current_list):
                # If listnumber is nan skip Object
                pass
            else:
                # If listnumber is a number add the UID to boxes
                boxes.detections[Current_list].UID = str(Object[self.OH.KnownOrder.get("UID")])
                if Current_list == TargetList:
                    # If Currentlist is the target change is_tracking to True and publish the pose
                    boxes.detections[Current_list].is_tracking = True

                    Pose = PoseStamped()
                    Pose.header = boxes.header
                    Pose.pose   = boxes.detections[Current_list].results[0].pose.pose
                    self.pose_pub.publish(Pose)

        self.Tracking_list.publish(boxes)

        time2 = rospy.Time.now().to_sec()
        print(time2-timer.time_ref.to_sec(),"Time delay")



        """
        if self.Target_Found == True:
            Target_I, Target_Occlusion = Find_target(self.OH, self.Target_UID)
            if Target_I == []:
                print("Target is Lost")
            elif Target_Occlusion > 0:
                print("Target was occluded {} frames ago".format(Target_Occlusion))
            else:
                Target = self.OH.Known[Target_I]
                SegID = Target[self.OH.KnownOrder.get("Current_listing")]
                #boxes.detections[SegID].is_tracking = True
        """

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
	ot = object_tracker()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)
