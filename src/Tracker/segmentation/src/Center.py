#!/usr/bin/env python3
import os
import sys
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import PointCloud2, Image, TimeReference
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

dir_to_Tracker=os.path.dirname(os.path.dirname(os.path.dirname( __file__ )))
dir_to_Scripts = os.path.join(dir_to_Tracker,"Scripts") 
sys.path.append(dir_to_Scripts)

from agfh import *

class Cloud_segmentation:
    def __init__(self):
        print("[INFO] Initializing ROS...")
        rospy.init_node("Segmentation")

        print("[INFO Loading modules...]")
        self.bridge = CvBridge()

        print("[INFO] Loading config...")
        self.time_list = []
        self.pc_list = []
        self.cv_image = []
        self.queue_lim = 30

        print("[INFO] Initializing ROS publishers...")
        self.bboxes_pub = rospy.Publisher("/Tracker/Segmentation/Boxes",Detection2DArray, queue_size=1) #Bboxes for object_handler 
        
        print("[INFO] Initialize ROS Subscribers...")
        cloud_sub = message_filters.Subscriber("/Tracker/Pc2ToImage/Cloud", Image, queue_size=1)
        timer_sub = message_filters.Subscriber("/Tracker/Timer", TimeReference, queue_size=1)

        print("[INFO] Loading complete")
        mf = message_filters.TimeSynchronizer([cloud_sub,timer_sub],queue_size=15)
        mf.registerCallback(self.callback_timer)

        self.callback_segmentation()


    def callback_segmentation(self):
        boxes = rospy.wait_for_message("/Tracker/Detection/Boxes",Detection2DArray)
        image = False
        i = 0
        for time in self.time_list:
            if time == boxes.header.stamp.to_sec():
                image = True
                cv_image = self.cv_image[i]
                break
            i += 1
        
        if image == True:
            PC_image_bbox_sub_series = []
            for box in boxes.detections:
                PC_image_bbox_sub = cv2.getRectSubPix(cv_image,(int(box.bbox.size_x),int(box.bbox.size_y)),(int(box.bbox.center.x),int(box.bbox.center.y))) # Extract bbox in depth image
                x = PC_image_bbox_sub[int(box.bbox.size_y/2)][int(box.bbox.size_x/2)][0]
                y = PC_image_bbox_sub[int(box.bbox.size_y/2)][int(box.bbox.size_x/2)][1]
                z = PC_image_bbox_sub[int(box.bbox.size_y/2)][int(box.bbox.size_x/2)][2]
                if np.isnan(x):
                    x = 1000
                    y = 1000
                    z = 1000

                box.results[0].pose.pose.position.x = x
                box.results[0].pose.pose.position.y = y
                box.results[0].pose.pose.position.z = z
                print(box.results[0].pose.pose.position)
            self.bboxes_pub.publish(boxes)
        self.callback_segmentation()
        #export ROS_MASTER_URI=http://localhost:11311
        #export ROS_IP=10.192.193.141

    def callback_timer(self,image,timer):
        cv_image    = np.array(self.bridge.imgmsg_to_cv2(image)) # Image with distance in channels x,y,z
        pc_list     = np.reshape(np.reshape(cv_image,(image.height,image.width*3)).T, image.height*image.width*3)
        self.time_list.append(image.header.stamp.to_sec())
        self.cv_image.append(cv_image)
        if len(self.time_list) > self.queue_lim:
            del self.time_list[0]
            del self.cv_image[0]

def main(args):
	segmentation = Cloud_segmentation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)