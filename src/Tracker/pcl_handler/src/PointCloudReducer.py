#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import cv2


dir_to_Tracker=os.path.dirname(os.path.dirname(os.path.dirname( __file__ )))
dir_to_Scripts = os.path.join(dir_to_Tracker,"Scripts") 
sys.path.append(dir_to_Scripts)
from agfh import *

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class PointCloudReducer:
    def __init__(self):
        rospy.init_node("PointCloudReducer")

        self.bridge = CvBridge()

        self.cloud_pub3 = rospy.Publisher("/Reduced_PointCloud",PointCloud2,queue_size=1)
        self.cloud_pub1 = rospy.Publisher("/Reduced_PointCloud_bb",PointCloud2,queue_size=1)
        self.cloud_pub2 = rospy.Publisher("/Reduced_PointCloud_seg",PointCloud2,queue_size=1)
        self.image_pub1 = rospy.Publisher("/Segmentation_image",Image,queue_size=1)

        image_sub = message_filters.Subscriber("/Tracker/Pc2ToImage/Cloud", Image, queue_size=1)
        boxes_sub = message_filters.Subscriber("/Tracker/Detection/Boxes", Detection2DArray, queue_size=1)
        cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2,queue_size=1)
        mf = message_filters.ApproximateTimeSynchronizer([image_sub,boxes_sub,cloud_sub],queue_size=10,slop=2)
        mf.registerCallback(self.callback)

    def callback(self,image,boxes,cloud):
        cv_image    = np.array(self.bridge.imgmsg_to_cv2(image), dtype='float32') # Image with distance in channels x,y,z
        
        PC_image_bbox_sub_series = []
        for box in boxes.detections:
            PC_image_bbox_sub = cv2.getRectSubPix(cv_image,(int(box.bbox.size_x),int(box.bbox.size_y)),(int(box.bbox.center.x),int(box.bbox.center.y))) # Extract bbox in depth image
            PC_image_bbox_sub_series.append(PC_image_bbox_sub)
        _, segmentation_img, xyzcoord_series, labels_series = DBSCAN_pointcloud(PC_image_bbox_sub_series, boxes.detections)

        pc_list, cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
        # bbox remove
       
        Start_x = int(boxes.detections[0].bbox.center.x-int(boxes.detections[0].bbox.size_x)/2)
        Start_y = int(boxes.detections[0].bbox.center.y-int(boxes.detections[0].bbox.size_y)/2)
        End_x   = int(boxes.detections[0].bbox.center.x+int(boxes.detections[0].bbox.size_x)/2)
        End_y   = int(boxes.detections[0].bbox.center.y+int(boxes.detections[0].bbox.size_y)/2)
        print(Start_x)
        print(End_x)
        bbox_i = []
        for y in range(Start_y,End_y):
            bbox_i += list(range((y*672+Start_x)*3,(y*672+End_x+1)*3))
        pc_list_bb = pc_list
        print(pc_list.shape)
        
        pc_list_bb = np.delete(pc_list_bb, bbox_i)
        pc_list_bb = pc_list_bb.reshape(int(len(pc_list_bb)/3),3)
        pc_list_bb = pc_list_bb[~np.isnan(pc_list_bb).any(axis=1)]
        pc_list_bb = pc_list_bb[~np.isinf(pc_list_bb).any(axis=1)]
        
        header = image.header
        Reduced_PC2_bb = pc2.create_cloud_xyz32(header, pc_list_bb)
        self.cloud_pub1.publish(Reduced_PC2_bb)
        
        bbox_i = []
        Seg_index = 0
        print(labels_series[0].shape)
        for y in range(Start_y,End_y):
            for x in range(Start_x,End_x):
                pc_index = (y*672+x)*3
                if labels_series[0][Seg_index] == True:
                    for i in [0,1,2]:
                        bbox_i.append(pc_index+i)
                Seg_index += 1
        print(y*x)
        print(Seg_index)
        pc_list_seg = pc_list
        pc_list_seg = np.delete(pc_list_seg, bbox_i)
        #print(pc_list.shape)
        pc_list_seg = pc_list_seg.reshape(int(len(pc_list_seg)/3),3)
        pc_list_seg= pc_list_seg[~np.isnan(pc_list_seg).any(axis=1)]
        pc_list_seg= pc_list_seg[~np.isinf(pc_list_seg).any(axis=1)]
        header = cloud.header
        Reduced_PC2_seg = pc2.create_cloud_xyz32(header, pc_list_seg)
        self.cloud_pub2.publish(Reduced_PC2_seg)

        imgmsg = self.bridge.cv2_to_imgmsg(segmentation_img[0])
        self.image_pub1.publish(imgmsg)
        #self.cloud_pub3(cloud)

def main(args):
    try:
        PointCloudReducer()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)

