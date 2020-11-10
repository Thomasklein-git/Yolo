#!/usr/bin/env python3
import sys
import rospy
import cv2

from sensor_msgs.msg import PointCloud2, Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import message_filters

from segmentation_utils import *

class Cloud_segmentation:
    def __init__(self):
        print("[INFO] Initializing ROS...")
        rospy.init_node("Segmentation", anonymous=True)

        print("[INFO] Loading modules...")
        self.bridge = CvBridge()
        
        print("[INFO] Loading config...")
        self.cloud_header = []
        self.cloud_pc_list = []
        self.cloud_cv_image_pc = []
        self.queue_lim = 10


        print("[INFO] Initializing ROS publisher...")
        self.bboxes_pub = rospy.Publisher("/yolo/Segbboxes",Detection2DArray, queue_size=1) #Bboxes for object_handler 
        self.images_pub = rospy.Publisher("/yolo/Segimages",Detection2DArray, queue_size=1) #Bboxes with images for display

        print("[INFO] Initialize ROS Subscribers...")
        #rospy.Subscriber("/zed2/zed_node/point_cloud/cloud_registered", PointCloud2, self.callback_cloud, queue_size=1)
        #rospy.Subscriber("/yolo/Timer", Image, self.callback_timer, queue_size=1)
        cloud_sub = message_filters.Subscriber("/zed2/zed_node/point_cloud/cloud_registered", PointCloud2, queue_size=1)
        timer_sub = message_filters.Subscriber("/yolo/Timer", Image, queue_size=1)
        mf = message_filters.TimeSynchronizer([cloud_sub,timer_sub],queue_size=15)
        mf.registerCallback(self.callback_timer)


        

        print("[INFO] Loading complete")
        self.callback()

        """

        print("[INFO] Loading ROS topics...")

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
        """
    def callback_timer(self,cloud,image):
        #print(cloud.header)
        time = rospy.Time.now().to_sec()
        pc_list = []
        for data in pc2.read_points(cloud, skip_nans=True):
            pass
            #pc_list.append([data[0], data[1], data[2], data[3]])
        pc_list, cv_image_pc = PC_dataxyz_to_PC_image(cloud,Org_img_height=376,Org_img_width=672)
        print(rospy.Time.now().to_sec()-time)
        
        self.cloud_header.append(cloud.header)
        self.cloud_pc_list.append(pc_list)
        self.cloud_cv_image_pc.append(cv_image_pc)
        if len(self.cloud_header) > self.queue_lim:
            del self.cloud_header[0]
            del self.cloud_pc_list[0]
            del self.cloud_cv_image_pc[0]
        """
        
    def callback(self):
        image = rospy.wait_for_message("/zed2/zed_node/left/image_rect_color",Image)
        """
        for 
        PC_image_bbox_sub_series = []
            for box in boxes.detections:
                PC_image_bbox_sub = cv2.getRectSubPix(cv_image_pc,(int(box.bbox.size_x),int(box.bbox.size_y)),(int(box.bbox.center.x),int(box.bbox.center.y))) # Extract bbox in depth image
                PC_image_bbox_sub_series.append(PC_image_bbox_sub)
            _, segmentation_img, xyzcoord_series, labels_series = DBSCAN_pointcloud(PC_image_bbox_sub_series, boxes.detections)
        """
        self.callback()
        
        

def main(args):
	#rospy.init_node('Cloud_segmentation', anonymous=True)
	segmentation = Cloud_segmentation()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ =='__main__':
	main(sys.argv)