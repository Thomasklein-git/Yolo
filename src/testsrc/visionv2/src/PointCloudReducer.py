#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import os
import time

import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, Image, TimeReference
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

class PointCloudReducer:
    def __init__(self):
        self.bridge = CvBridge()

        self.cloud_pub = rospy.Publisher("/Reduced_cloud", PointCloud2, queue_size=1)
		
        cloud_sub = message_filters.Subscriber("/yolo/CloudImage", Image, queue_size=1)
        boxes_sub = message_filters.Subscriber("/yolo/Trackedbboxes", Detection2DArray, queue_size=1)
        timer_sub = message_filters.Subscriber("/yolo/Timer", TimeReference, queue_size=1)
        
        mf = message_filters.TimeSynchronizer([cloud_sub,boxes_sub,timer_sub],queue_size=30)
        mf.registerCallback(self.callback)

    def callback(self,cloud,boxes,timer):
        Target = []
        for Object in boxes.detections:
            if Object.is_tracking == True:
                Target = Object
                break
        time1 = rospy.Time.now().to_sec()
        Reduced_PC2 = self.PC_reduc_sep(Target,cloud)
        self.cloud_pub.publish(Reduced_PC2)
        time2 = rospy.Time.now().to_sec()
        print(time2-time1, "Time")
        #print(time2-timer.time_ref.to_sec(),"Time delay")

    
    def PC_reduc_sep(self,bbox,CloudImage):
        cv_image    = np.array(self.bridge.imgmsg_to_cv2(CloudImage), dtype='float32') # Image with distance in channels x,y,z
        pc_list     = np.reshape(np.reshape(cv_image,(CloudImage.height,CloudImage.width*3)).T, CloudImage.height*CloudImage.width*3)
        if bbox != []:
            cx = bbox.bbox.center.x
            sx = bbox.bbox.size_y
            cy = bbox.bbox.center.y
            sy = bbox.bbox.size_y
            Start_x = int(cx - sx/2)
            End_x   = int(cx + sx/2)
            Start_y = int(cy - sy/2)
            End_y   = int(cy + sy/2)

            bbox_i = []
            time1 = rospy.Time.now().to_sec()
            for y in range(Start_y,End_y):
                bbox_i += list(range((y*672+Start_x)*3,(y*672+End_x+1)*3))
            pc_list = np.delete(pc_list, bbox_i)
            time2 = rospy.Time.now().to_sec()
            print(time1-time2, "Time1")
        time3 = rospy.Time.now().to_sec()
        pc_list = pc_list.reshape(int(len(pc_list)/3),3)
        pc_list = pc_list[~np.isnan(pc_list).any(axis=1)]
        pc_list = pc_list[~np.isinf(pc_list).any(axis=1)]
        time4 = rospy.Time.now().to_sec()
        print(time4-time3, "Time2")
        
        print(pc_list.shape)

        Reduced_PC2 = pc2.create_cloud_xyz32(CloudImage.header, pc_list)
        time5 = rospy.Time.now().to_sec()
        print(time5-time4)
        return Reduced_PC2

def main(args):
	rospy.init_node('PointCloudReducer', anonymous=True)
	pcr = PointCloudReducer()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows

if __name__ =='__main__':
	main(sys.argv)