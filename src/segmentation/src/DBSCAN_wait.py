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
        print("[INFO] initializing config...")
        self.publish_images = True
        self.bridge = CvBridge()
        
        print("[INFO] Loading ROS topics...")
        self.bboxes_pub = rospy.Publisher("/yolo/Segbboxes",Detection2DArray, queue_size=1) #Bboxes for object_handler 
        self.images_pub = rospy.Publisher("/yolo/Segimages",Detection2DArray, queue_size=1) #Bboxes with images for display

        cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2)
        self.cache = message_filters.Cache(cloud_sub,30)

        #boxes_sub = message_filters.Subscriber("/yolo/bboxes",Detection2DArray)

        #rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2, self.callback_cloud, queue_size = 30)
        rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered",PointCloud2, self.callback_cloud, queue_size=1)
        rospy.Subscriber("/yolo/bboxes",Detection2DArray,self.callback_box, queue_size=1)

        print("[INFO] Loading complete")
        self.cloud_list = []
        self.cloud_list_headers = []
        self.queue_lim = 60
        #mf = message_filters.TimeSynchronizer([cloud_sub,boxes_sub],30)
        #mf.registerCallback(self.callback)
        
    def callback_cloud(self, cloud):

        self.cloud_list.append(cloud)
        self.cloud_list_headers.append(cloud.header)
        if len(self.cloud_list) > self.queue_lim:
            del self.cloud_list[0]
            del self.cloud_list_headers[0]

    def callback_box(self, boxes):
        i = 0
        cloud = []
        for heads in self.cloud_list_headers:
            print(heads.stamp.to_sec() - boxes.header.stamp.to_sec())
            if heads.stamp.to_sec() == boxes.header.stamp.to_sec():
                cloud = self.cloud_list[i]
                print("things happen")
            i += 1
        if cloud != []:
                #cloud = self.cache.getElemAfterTime(boxes.header.stamp)
            print("in loop")
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
            print("Waiting")

            self.bboxes_pub.publish(boxes) 
        else:
            print("Point missed")
        #print(cloud)
        #self.bboxes_pub.publish(boxes) 
        
        

def main(args):
	rospy.init_node('Cloud_segmentation', anonymous=True)
	segmentation = Cloud_segmentation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)