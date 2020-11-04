#!/usr/bin/env python3
import sys
import rospy
import cv2

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from segmentation_utils import *

class Cloud_segmentation:
    def __init__(self):
        self.bridge = CvBridge()

        self.boxes_pub = rospy.Publisher("/segmented/bboxes",Detection2DArray, queue_size=1)

        cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)
        boxes_sub = message_filters.Subscriber("/yolo/bboxes",Detection2DArray)

        mf = message_filters.TimeSynchronizer([cloud_sub,boxes_sub,],10)
        mf.registerCallback(self.callback)

    def callback(self, cloud, boxes):
        pc_list, cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
        PC_image_bbox_sub_series = []
        for box in boxes.detections:
            PC_image_bbox_sub = cv2.getRectSubPix(cv_image_pc,(int(box.bbox.size_x),int(box.bbox.size_y)),(int(box.bbox.center.x),int(box.bbox.center.y))) # Extract bbox in depth image
            PC_image_bbox_sub_series.append(PC_image_bbox_sub)
        _, segmentation_img, xyzcoord_series, labels_series = DBSCAN_pointcloud(PC_image_bbox_sub_series, boxes.detections)
        
        for i in range(0,len(boxes.detections)):
            boxes.detections[i].results[0].pose.pose.position.x = xyzcoord_series[i][0]
            boxes.detections[i].results[0].pose.pose.position.y = xyzcoord_series[i][1]
            boxes.detections[i].results[0].pose.pose.position.z = xyzcoord_series[i][2]
        
        self.boxes_pub.publish(boxes)  
        

def main(args):
	rospy.init_node('Cloud_segmentation', anonymous=True)
	segmentation = Cloud_segmentation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)